import torch.nn as nn
import torch

class ADModel(nn.Module):
    def __init__(self):

        super().__init__()

        self.fv_feture_extractor = nn.Sequential(
            nn.Conv2d(3, 32, 3, 2),
            nn.ReLU(),
            nn.AvgPool2d(4),
            nn.Conv2d(32, 32, 3, 2),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Flatten()
        )
        self.speed_feture_extractor = nn.Sequential(
            nn.Linear(1, 16),
            nn.ReLU(),
            nn.Linear(16, 32)
        )

        self.longitudinal_head = nn.Sequential(
            nn.Linear(256+32, 32),
            nn.ReLU(),
            nn.Linear(32, 8),
            nn.ReLU(),
            nn.Linear(8, 1),
            nn.Tanh()
        )
        self.lateral_head = nn.Sequential(
            nn.Linear(256+32, 32),
            nn.ReLU(),
            nn.Linear(32, 8),
            nn.ReLU(),
            nn.Linear(8, 1),
            nn.Tanh()
        )
        

    def forward(self, fv, speed):

        fv_fe = self.fv_feture_extractor(fv)
        speed_fe = self.speed_feture_extractor(speed)
        fe = torch.cat([fv_fe, speed_fe], 1)
        pedal = self.longitudinal_head(fe)
        steer = self.lateral_head(fe)

        return pedal, steer

if __name__ == '__main__':
    device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
    print(device)
    model = ADModel().to(device)

    fv = torch.ones([5, 3, 150, 100]).to(device)
    speed = torch.ones([5, 1]).to(device)

    pedal, steer = model(fv, speed)
    print(pedal, steer)
