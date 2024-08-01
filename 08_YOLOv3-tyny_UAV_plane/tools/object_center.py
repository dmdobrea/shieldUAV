from pynq_dpu import DpuOverlay

import numpy as np
import cv2
import dlib
import imutils
import colorsys
import threading

import time

from matplotlib.patches import Rectangle

job_id = 0
job_id_ready = False

class ObjCenter:
	def __init__(self, yoloPath):
		# prepare for more than one
		self.prev_top    = None
		self.prev_left   = None
		self.prev_bottom = None
		self.prev_right  = None
		
		print("[INFO_OC] : YOLO object creation\n")
		
		self.overlay = DpuOverlay("dpu.bit")
		
		self.overlay.load_model(yoloPath)

		anchor_list  = [11, 11, 15, 15, 19, 18, 23, 23, 30, 29, 42, 38] 	# for YOLOv3-tiny
				
		anchor_float = [float(x) for x in anchor_list]
		self.anchors = np.array(anchor_float).reshape(-1, 2)
		
		classes_path = "voc_classes.txt"
		self.class_names  = self.get_class(classes_path)
		
		num_classes = len(self.class_names)
		
		self.dpu      = self.overlay.runner
		inputTensors  = self.dpu.get_input_tensors()
		outputTensors = self.dpu.get_output_tensors()

		self.shapeIn     = tuple(inputTensors[0].dims)

		self.shapeOut0   = (tuple(outputTensors[0].dims)) # (1, 13, 13, 75)
		self.shapeOut1   = (tuple(outputTensors[1].dims)) # (1, 26, 26, 75)
		#self.shapeOut2   = (tuple(outputTensors[2].dims)) # (1, 52, 52, 75)   #DDM

		outputSize0 = int(outputTensors[0].get_data_size() / self.shapeIn[0]) # 12675
		outputSize1 = int(outputTensors[1].get_data_size() / self.shapeIn[0]) # 50700
		#outputSize2 = int(outputTensors[2].get_data_size() / self.shapeIn[0]) # 202800  #DDM

		self.input_data   = [np.empty(self.shapeIn,   dtype=np.float32, order="C")]
		self.output_data  = [np.empty(self.shapeOut0, dtype=np.float32, order="C"), 
			np.empty(self.shapeOut1, dtype=np.float32, order="C")]
			# np.empty(self.shapeOut2, dtype=np.float32, order="C")]   #DDM

		self.image = self.input_data[0]
		
	def __del__(self):
		print("[INFO_OC] : YOLO object destruction\n\n")
		del self.overlay
		del self.dpu		

	'''Get model classification information'''	
	def get_class(self, classes_path):
		with open(classes_path) as f:
			class_names = f.readlines()
		class_names = [c.strip() for c in class_names]
		return class_names
		
	'''resize image with unchanged aspect ratio using padding'''
	def letterbox_image(self, image, size):
		ih, iw, _ = image.shape
		w, h = size
		scale = min(w/iw, h/ih)
		#print(scale)
		
		nw = int(iw*scale)
		nh = int(ih*scale)
		#print(nw)
		#print(nh)

		image = cv2.resize(image, (nw,nh), interpolation=cv2.INTER_LINEAR)
		new_image = np.ones((h,w,3), np.uint8) * 128
		h_start = (h-nh)//2
		w_start = (w-nw)//2
		new_image[h_start:h_start+nh, w_start:w_start+nw, :] = image
		return new_image
		
	'''image preprocessing'''
	def pre_process(self, image, model_image_size):
		image = image[...,::-1]
		image_h, image_w, _ = image.shape
	 
		if model_image_size != (None, None):
			assert model_image_size[0]%32 == 0, 'Multiples of 32 required'
			assert model_image_size[1]%32 == 0, 'Multiples of 32 required'
			boxed_image = self.letterbox_image(image, tuple(reversed(model_image_size)))
		else:
			new_image_size = (image_w - (image_w % 32), image_h - (image_h % 32))
			boxed_image = self.letterbox_image(image, new_image_size)
		image_data = np.array(boxed_image, dtype='float32')
		image_data /= 255.0
		image_data = np.expand_dims(image_data, 0) 	
		return image_data
		
	def _get_feats(self, feats, anchors, num_classes, input_shape):
		num_anchors = len(anchors)
		anchors_tensor = np.reshape(np.array(anchors, dtype=np.float32), [1, 1, 1, num_anchors, 2])
		grid_size = np.shape(feats)[1:3]
		nu = num_classes + 5
		predictions = np.reshape(feats, [-1, grid_size[0], grid_size[1], num_anchors, nu])
		grid_y = np.tile(np.reshape(np.arange(grid_size[0]), [-1, 1, 1, 1]), [1, grid_size[1], 1, 1])
		grid_x = np.tile(np.reshape(np.arange(grid_size[1]), [1, -1, 1, 1]), [grid_size[0], 1, 1, 1])
		grid = np.concatenate([grid_x, grid_y], axis = -1)
		grid = np.array(grid, dtype=np.float32)

		box_xy = (1/(1+np.exp(-predictions[..., :2])) + grid) / np.array(grid_size[::-1], dtype=np.float32)
		box_wh = np.exp(predictions[..., 2:4]) * anchors_tensor / np.array(input_shape[::-1], dtype=np.float32)
		box_confidence = 1/(1+np.exp(-predictions[..., 4:5]))
		box_class_probs = 1/(1+np.exp(-predictions[..., 5:]))
		return box_xy, box_wh, box_confidence, box_class_probs
		
	def correct_boxes(self, box_xy, box_wh, input_shape, image_shape):
		box_yx = box_xy[..., ::-1]
		box_hw = box_wh[..., ::-1]
		input_shape = np.array(input_shape, dtype = np.float32)
		image_shape = np.array(image_shape, dtype = np.float32)
		new_shape = np.around(image_shape * np.min(input_shape / image_shape))
		offset = (input_shape - new_shape) / 2. / input_shape
		scale = input_shape / new_shape
		box_yx = (box_yx - offset) * scale
		box_hw *= scale

		box_mins = box_yx - (box_hw / 2.)
		box_maxes = box_yx + (box_hw / 2.)
		boxes = np.concatenate([
			box_mins[..., 0:1],
			box_mins[..., 1:2],
			box_maxes[..., 0:1],
			box_maxes[..., 1:2]
		], axis = -1)
		boxes *= np.concatenate([image_shape, image_shape], axis = -1)
		return boxes
		
	def boxes_and_scores(self, feats, anchors, classes_num, input_shape, image_shape):
		box_xy, box_wh, box_confidence, box_class_probs = self._get_feats(feats, anchors, classes_num, input_shape)
		boxes = self.correct_boxes(box_xy, box_wh, input_shape, image_shape)
		boxes = np.reshape(boxes, [-1, 4])
		box_scores = box_confidence * box_class_probs
		box_scores = np.reshape(box_scores, [-1, classes_num])
		return boxes, box_scores
		
	def nms_boxes(self, boxes, scores):
		"""Suppress non-maximal boxes.

		# Arguments
			boxes: ndarray, boxes of objects.
			scores: ndarray, scores of objects.

		# Returns
			keep: ndarray, index of effective boxes.
		"""
		x1 = boxes[:, 0]
		y1 = boxes[:, 1]
		x2 = boxes[:, 2]
		y2 = boxes[:, 3]

		areas = (x2-x1+1)*(y2-y1+1)
		order = scores.argsort()[::-1]

		keep = []
		while order.size > 0:
			i = order[0]
			keep.append(i)

			xx1 = np.maximum(x1[i], x1[order[1:]])
			yy1 = np.maximum(y1[i], y1[order[1:]])
			xx2 = np.minimum(x2[i], x2[order[1:]])
			yy2 = np.minimum(y2[i], y2[order[1:]])

			w1 = np.maximum(0.0, xx2 - xx1 + 1)
			h1 = np.maximum(0.0, yy2 - yy1 + 1)
			inter = w1 * h1

			ovr = inter / (areas[i] + areas[order[1:]] - inter)
			inds = np.where(ovr <= 0.55)[0]  # threshold
			order = order[inds + 1]

		return keep		
		
	def evaluate(self, yolo_outputs, image_shape, class_names, anchors):
		score_thresh = 0.2
		#anchor_mask = [[6, 7, 8], [3, 4, 5], [0, 1, 2]]
		anchor_mask = [[3, 4, 5], [0, 1, 2]]
		boxes = []
		box_scores = []
		input_shape = np.shape(yolo_outputs[0])[1 : 3]
		input_shape = np.array(input_shape)*32
		
		for i in range(len(yolo_outputs)):
			_boxes, _box_scores = self.boxes_and_scores(
				yolo_outputs[i], anchors[anchor_mask[i]], len(class_names), 
				input_shape, image_shape)
			boxes.append(_boxes)
			box_scores.append(_box_scores)
		boxes = np.concatenate(boxes, axis = 0)
		box_scores = np.concatenate(box_scores, axis = 0)

		mask = box_scores >= score_thresh
		boxes_ = []
		scores_ = []
		classes_ = []
		for c in range(len(class_names)):
			class_boxes_np = boxes[mask[:, c]]
			class_box_scores_np = box_scores[:, c]
			class_box_scores_np = class_box_scores_np[mask[:, c]]
			nms_index_np = self.nms_boxes(class_boxes_np, class_box_scores_np) 
			class_boxes_np = class_boxes_np[nms_index_np]
			class_box_scores_np = class_box_scores_np[nms_index_np]
			classes_np = np.ones_like(class_box_scores_np, dtype = np.int32) * c
			boxes_.append(class_boxes_np)
			scores_.append(class_box_scores_np)
			classes_.append(classes_np)
		boxes_   = np.concatenate(boxes_,  axis = 0)
		scores_  = np.concatenate(scores_, axis = 0)
		classes_ = np.concatenate(classes_, axis = 0)

		return boxes_, scores_, classes_
	
	
	def dpu_run(self, input_data, output_data):
		global job_id
		global job_id_ready
		job_id = self.dpu.execute_async(input_data, output_data)
		job_id_ready = True
	
	
	""" The main function were the detection is done """
	def update(self, input_image, frameCenter, selected_object = 1):
		global job_id_ready
		global job_id
		# Pre-processing
		image_size = input_image.shape[:2]
		image_data = np.array(self.pre_process(input_image, (640, 480)), dtype=np.float32)
		
		# Fetch data to DPU and trigger it
		self.image[0,...] = image_data.reshape(self.shapeIn[1:])

		start = time.time()
		th = threading.Thread(target=self.dpu_run, args=(self.input_data, self.output_data))
		th.start()
		# starting from here the system wait the DPU to finish
		if self.prev_top != None:
			# update the tracker and grab the updated position
			self.tracker.update(input_image)
			pos = self.tracker.get_position()

			# unpack the position object
			left_cta   = int(pos.left())
			top_cta    = int(pos.top())
			right_cta  = int(pos.right())
			bottom_cta = int(pos.bottom())	
		
		while not job_id_ready:
			pass
		self.dpu.wait(job_id)
		th.join()
		job_id_ready = False
		# => the wait is finished
		stop = time.time()	
		
		print ("Durata procesare =", stop-start, "[s]")
		
		# Retrieve output data
		conv_out0 = np.reshape(self.output_data[0], self.shapeOut0)
		conv_out1 = np.reshape(self.output_data[1], self.shapeOut1)
		# conv_out2 = np.reshape(self.output_data[2], self.shapeOut2)  #DDM
		
		yolo_outputs = [conv_out0, conv_out1]
		# yolo_outputs = [conv_out0, conv_out1, conv_out2]   #DDM
		
		# Decode output from YOLO
		boxes, scores, classes = self.evaluate(yolo_outputs, image_size, self.class_names, self.anchors)

		founded_UAV = 0
		
		# search from the detection object only the first object of type "selected_object"
		for i, bbox in enumerate(boxes):
			score, class_index = scores[i], classes[i]
			if class_index == selected_object:
				founded_UAV = 1
				
				[top, left, bottom, right] = bbox
				width, height = right - left, bottom - top
				center_x, center_y = left + width*0.5, top + height*0.5
				
				# for the correlation tracker
				self.prev_top    = top
				self.prev_left   = left
				self.prev_bottom = bottom
				self.prev_right  = right   
				
				# preparing a potetial frame without any detection
				#====================================================
				# construct a dlib rectangle object from the bounding
				# box coordinates and then start the dlib correlation
				# tracker
				self.tracker = dlib.correlation_tracker()
				rect = dlib.rectangle(left, top, right, bottom)
				self.tracker.start_track(input_image, rect)
			
				return ((center_x, center_y), bbox, 1)
				
		if founded_UAV == 0 and self.prev_top != None:
			width_cta, height_cta = right_cta - left_cta, bottom_cta - top_cta
			center_x_cta, center_y_cta = left_cta + width_cta*0.5, top_cta + height_cta*0.5
			
			return ((center_x_cta, center_y_cta), [top_cta, left_cta, bottom_cta, right_cta], 2)

		return (frameCenter, None, 0)
