import cv2
import numpy as np
import torch
import torch.backends.cudnn as cudnn

from .data import cfg
from .layers.functions.prior_box import PriorBox
from .models.faceboxes import FaceBoxes
from .utils.box_utils import decode
from .utils.nms.py_cpu_nms import py_cpu_nms


class FaceDetector:
    def __init__(self, weights_path, conf_th=.6, nms_th=.3):
        self.conf_th = conf_th
        self.nms_th = nms_th
        torch.set_grad_enabled(False)
        # net and model
        net = FaceBoxes(phase='test', size=None, num_classes=2)    # initialize detector
        net = load_model(net, weights_path)
        net.eval()
        cudnn.benchmark = True
        self.device = torch.device("cpu")
        self.net = net.to(self.device)

    def detect_face(self, img):
        # testing begin
        img = np.float32(img)
        im_height, im_width, _ = img.shape
        scale = torch.Tensor([img.shape[1], img.shape[0], img.shape[1], img.shape[0]])
        img -= (104, 117, 123)
        img = img.transpose(2, 0, 1)
        img = torch.from_numpy(img).unsqueeze(0)
        img = img.to(self.device)
        scale = scale.to(self.device)

        loc, conf = self.net(img)  # forward pass
        priorbox = PriorBox(cfg, image_size=(im_height, im_width))
        priors = priorbox.forward()
        priors = priors.to(self.device)
        prior_data = priors.data
        boxes = decode(loc.data.squeeze(0), prior_data, cfg['variance'])
        boxes = boxes * scale
        boxes = boxes.cpu().numpy()
        scores = conf.squeeze(0).data.cpu().numpy()[:, 1]

        # ignore low scores
        inds = np.where(scores > self.conf_th)[0]
        boxes = boxes[inds]
        scores = scores[inds]

        # do NMS
        dets = np.hstack((boxes, scores[:, np.newaxis])).astype(np.float32, copy=False)
        keep = py_cpu_nms(dets, self.nms_th)
        # keep = nms(dets, args.nms_threshold, force_cpu=not args.gpu)
        return dets[keep]


def check_keys(model, pretrained_state_dict):
    ckpt_keys = set(pretrained_state_dict.keys())
    model_keys = set(model.state_dict().keys())
    used_pretrained_keys = model_keys & ckpt_keys
    unused_pretrained_keys = ckpt_keys - model_keys
    missing_keys = model_keys - ckpt_keys
    # print('Missing keys: {}'.format(len(missing_keys)))
    # print('Unused checkpoint keys: {}'.format(len(unused_pretrained_keys)))
    # print('Used keys: {}'.format(len(used_pretrained_keys)))
    assert len(used_pretrained_keys) > 0, 'load NONE from pretrained checkpoint'


def remove_prefix(state_dict, prefix):
    ''' Old style model is stored with all names of parameters sharing common prefix 'module.' '''
    def f(x): return x.split(prefix, 1)[-1] if x.startswith(prefix) else x
    return {f(key): value for key, value in state_dict.items()}


def load_model(model, pretrained_path):
    print('Loading pretrained model from {}'.format(pretrained_path))
    pretrained_dict = torch.load(pretrained_path, map_location=lambda storage, loc: storage)
    pretrained_dict = remove_prefix(pretrained_dict, 'module.')
    check_keys(model, pretrained_dict)
    model.load_state_dict(pretrained_dict, strict=False)
    return model
