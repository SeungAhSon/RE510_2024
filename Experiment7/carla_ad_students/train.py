from dataset import FVDataset
from model.ADModel import ADModel
import torch
import datetime
import os
from torch.utils.data import DataLoader
import torch.optim as optim
import tqdm
import torch.nn.functional as F

if __name__ == "__main__":

    device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
    timestamp = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
    logdir = os.path.join("weights", timestamp)
    os.makedirs(logdir)

    model = ADModel().to(device)
    dataset = FVDataset('fv_out')
    train_size = int(0.9 * len(dataset))
    valid_size = len(dataset) - train_size
    batch_size = 32
    learning_rate = 0.0001
    epoch = 100

    torch.manual_seed(42)
    train_ds, valid_ds = torch.utils.data.random_split(dataset, [train_size, valid_size])

    train_dl = DataLoader(train_ds, batch_size, shuffle=True)
    valid_dl = DataLoader(valid_ds, batch_size, shuffle=False)
    optimizer = optim.Adam(model.parameters(), learning_rate)

    best_train = float("Inf")
    best_valid = float("Inf")

    for epoch in tqdm.tqdm(range(epoch), "Epoch", position=1):
        epoch_train_loss = 0
        epoch_valid_loss = 0

        model.train()
        for batch in tqdm.tqdm(train_dl, "Training Batch", position=0, leave=False):

            fv_batch, speed_batch, pedal_batch, steer_batch = batch
            cbs = int(fv_batch.shape[0])

            fv_batch, speed_batch, pedal_batch, steer_batch = fv_batch.to(device), speed_batch.to(device), pedal_batch.to(device), steer_batch.to(device)
            
            optimizer.zero_grad()
            pedal_pred, steer_pred = model(fv_batch, speed_batch)
            # pedal_pred = torch.reshape(pred, (-1, CONFIG["HORIZON"], 2))
            pedal_loss = F.mse_loss(pedal_pred, pedal_batch)
            steer_loss = F.mse_loss(steer_pred, steer_batch)
            loss = pedal_loss + steer_loss

            loss.backward()
            optimizer.step()

            epoch_train_loss += (loss.item() * cbs)

        model.eval()
        with torch.no_grad():
            for batch in tqdm.tqdm(valid_dl, "Validation Batch", position=0, leave=False):

                fv_batch, speed_batch, pedal_batch, steer_batch = batch
                cbs = int(fv_batch.shape[0])
                fv_batch, speed_batch, pedal_batch, steer_batch = fv_batch.to(device), speed_batch.to(device), pedal_batch.to(device), steer_batch.to(device)
                pedal_pred, steer_pred = model(fv_batch, speed_batch)
                # pred = torch.reshape(pred, (-1, CONFIG["HORIZON"], 2))
                pedal_loss = F.mse_loss(pedal_pred, pedal_batch)
                steer_loss = F.mse_loss(steer_pred, steer_batch)
                loss = 0.3*pedal_loss + 0.7*steer_loss

                epoch_valid_loss += (loss.item()) * cbs

        epoch_train_loss = epoch_train_loss / train_size
        epoch_valid_loss = epoch_valid_loss / valid_size

        tqdm.tqdm.write("Training loss: %.4f || Validation loss: %.4f " 
                         % (epoch_train_loss, epoch_valid_loss))

        chkpt = {
            "model": model.state_dict(),
            "optimizer": optimizer.state_dict(),
            "train_loss": epoch_train_loss,
            "valid_loss": epoch_valid_loss,
            "epoch": epoch
        }

        torch.save(chkpt, os.path.join(logdir, "latest.pt"))
