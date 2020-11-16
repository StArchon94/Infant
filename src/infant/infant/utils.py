import cv2
import numpy as np
import torch
from torchvision.transforms.functional import to_pil_image, to_tensor


def pixelate(img, l):
    t = (l / 100)**0.1
    s = 1 - (1 - 0.02) * t
    img_small = cv2.resize(img, None, fx=s, fy=s)
    return cv2.resize(img_small, img.shape[1::-1], interpolation=cv2.INTER_NEAREST)


class Squeezer:
    def __init__(self, h, w, x0=0, y0=0):
        if torch.cuda.is_available():
            torch.set_default_tensor_type(torch.cuda.FloatTensor)
            self.dev = 'cuda'
        else:
            torch.set_default_tensor_type(torch.FloatTensor)
            self.dev = 'cpu'

        self.x0 = x0
        self.y0 = y0
        y, x = torch.meshgrid(torch.linspace(-1, 1, w), torch.linspace(-1, 1, h))
        self.dx = x - x0
        self.dy = y - y0
        self.r2 = self.dx**2 + self.dy**2

    def squeeze(self, img, l):
        img = to_tensor(img).to(self.dev).unsqueeze(0)
        exp = 1 - l / 200
        c = torch.pow(self.r2, (exp - 1) / 2)
        xp = self.x0 + c * self.dx
        yp = self.y0 + c * self.dy
        grid = torch.stack([xp, yp], axis=-1).unsqueeze(0)
        squeezed = torch.nn.functional.grid_sample(img, grid, padding_mode='border')
        return np.asarray(to_pil_image(squeezed.squeeze(0)))
