#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Generative Adversarial Nets (GAN) using pytorch
#
im
import torch
from torch.utils.data import DataLoader
from torchvision import transforms
from torchvision import datasets

class ToImg(nn.Module):
    def forward(self, x):
        n, c = x.shape
        return x.reshape(n,128,4,4)

class Generator(nn.Module):
    def __init__(self):
        super().__init__()

        self.layers = nn.Sequential(
            nn.Linear(64, 128*4*4),
            nn.BatchNorm1d(128*4*4),
            nn.ReLU(inplace=True),
            ToImg(),
            # 4x4 -> 16x16
            nn.ConvTranspose2d(128, 128, kernel_size=8, stride=4, padding=2),
            nn.BatchNorm2d(128),
            nn.ReLU(inplace=True),
            # 16x16 -> 32x32
            nn.ConvTranspose2d(128, 64, kernel_size=4, stride=2, padding=1),
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),
            # 32x32 -> 64x64
            nn.ConvTranspose2d(64, 3, kernel_size=4, stride=2, padding=1),
            nn.Sigmoid(),
        )

    def forward(self, x):
        return self.layers(x)

class Discriminator(nn.Module):
    def __init__(self):
        super().__init__()

    self.layers = nn.Sequential(
            # 64x64 -> 32x32
            nn.Conv2d(3, 64, kernel_size=4, stride=2, padding=1),
            nn.LeakyReLU(0.2, inplace=True),
            # 32x32 -> 16x16
            nn.Conv2d(64, 128, kernel_size=4, stride=2, padding=1),
            nn.BatchNorm2d(128),
            nn.LeakyReLU(0.2, inplace=True),
            # 16x16 -> 4x4
            nn.Conv2d(128, 256, kernel_size=8, stride=4, padding=2),
            nn.BatchNorm2d(256),
            nn.LeakyReLU(0.2, inplace=True),
            # 4x4 -> 1x1
            nn.Conv2d(256, 512, kernel_size=8, stride=4, padding=2),
            nn.BatchNorm2d(512),
            nn.LeakyReLU(0.2, inplace=True),
            nn.Conv2d(512, 1, kernel_size=1, stride=1, padding=0),
            nn.Flatten(),
            nn.Sigmoid(),
        )

    def forward(self, x):
        return self.layers(x)

if __name__ == '__main__':

    device = 'cuda' if torch.cuda.is_available() else 'cpu'

    # model
    netG = Generator().to(device)
    netD = Discriminator().to(device)
    optG = torch.optim.Adam(netG.parameters(), lr=0.0002, betas=(0.5, 0.999))
    optD = torch.optim.Adam(netD.parameters(), lr=0.0002, betas=(0.5, 0.999))
    criterion = torch.nn.BCELoss()

    # dataset
    transform = transforms.Compose([transforms.CenterCrop(160),transforms.Resize((64,64)),transforms.ToTensor(), ])
    trainset = datasets.CelebA('~/data', download=True, split='train', transform=transform)
    bs = 16
    train_loader = DataLoader(trainset, batch_size=bs, shuffle=True)

    # training
    nepoch = 10
    losses = []
    for iepoch in range(nepoch):
        for i, data in enumerate(train_loader):
            x, y = data
            x = x.to(device)

            # train generator
            z = torch.randn(bs, 64).to(x.device)
            x_ = netG.forward(z)
            d_ = netD.forward(x_) # fake
            lossG = criterion(d_, torch.ones_like(d_))

            optG.zero_grad()
            lossG.backward()
            optG.step()

            # train discriminator
            z = torch.randn(x.shape[0], 64).to(x.device)
            x_ = netG.forward(z)
            d = netD.forward(x)   # real
            d_ = netD.forward(x_) # fake
            loss_real = criterion(d, torch.ones_like(d))
            loss_fake = criterion(d_, torch.zeros_like(d_))
            lossD = (loss_real + loss_fake)/2.

            optD.zero_grad()
            lossD.backward()
            optD.step()

            print('ep: %02d %04d lossG=%.10f lossD=%.10f' % (iepoch, i, lossG.item(), lossD.item()))
            losses.append([lossG.item(), lossD.item()])

        # output generated images
        netG.eval()
        z = torch.randn(32, 64).to(x.device)
        x_ = netG.forward(z)
        dst = x_.to('cpu').detach().numpy()
        dst = dst.reshape(4,8,3,64,64)
        dst = dst.transpose(0,3,1,4,2)
        dst = dst.reshape(4*64,8*64,3)
        dst = np.clip(dst*255., 0, 255).astype(np.uint8)
        skio.imsave('out/img_%03d.png' % iepoch, dst)

        # output loss plots
        losses_ = np.array(losses)
        niter = losses_.shape[0]//100*100
        x_iter = np.arange(100)*(niter//100) + niter//200
        plt.plot(x_iter, losses_[:niter,0].reshape(100,-1).mean(1))
        plt.plot(x_iter, losses_[:niter,1].reshape(100,-1).mean(1))
        plt.tight_layout()
        plt.savefig('out/loss_%03d.png' % iepoch)
        plt.clf()
        netG.train()
