import torch

from glob import glob
import os
import numpy as np
import cv2
from torch.utils.data import ConcatDataset
from torch.utils.data import DataLoader
import tqdm


class FVDataset(torch.utils.data.Dataset):
    def __init__(self, path):
        self.fv_path_list = sorted(glob(path+'/*/*.png'))

    def __len__(self):
        return len(self.fv_path_list)

    def __getitem__(self, idx):
        fv_path = self.fv_path_list[idx]
        fv_img = cv2.imread(fv_path)
        fv_img = cv2.resize(fv_img, [150, 100]).transpose(2, 0, 1)

        filename = os.path.splitext(os.path.basename(fv_path))[0]
        _, _, speed, _, pedal, _, steer = filename.split('_')
        speed = float(speed)
        pedal = float(pedal)
        steer = float(steer)


        return fv_img.astype(np.float32), np.array([speed], dtype=np.float32), np.array([pedal], dtype=np.float32), np.array([steer], dtype=np.float32)

if __name__ == '__main__':
    fvdataset = FVDataset('fv_out')
    dataloader = DataLoader(fvdataset, batch_size=2, shuffle=True)

    for batch in dataloader:
        (fv_image, data) = batch
        fv_image = (fv_image[0].numpy()).astype(np.uint8)
        print(data[0])

        cv2.imshow("fv_image", fv_image)
        cv2.waitKey(0)