#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Complex Number CNN using pytorch
#
import pandas as pd
import torch
from torch import nn
from torch import optim

class ComplexConv2d(nn.Module):
    def __init__(self, n_channels, out_channels, kernel_size, stride=1, padding=0, dilation=1, groups=1, bias=True):
        super().__init__()
        self.conv_real = nn.Conv2d(n_channels, out_channels, kernel_size, stride, padding, dilation, groups, bias)
        self.conv_imag = nn.Conv2d(n_channels, out_channels, kernel_size, stride, padding, dilation, groups, bias)

    def forward(self, x):
        if len(x.shape)==4: # n,c,w,h
            x_real = x
            x_imag = torch.zeros_like(x)
        else: # n,p,c,w,h
            x_real = x[:,0]
            x_imag = x[:,1]

        x_real_out = self.conv_real(x_real) - self.conv_imag(x_imag)
        x_imag_out = self.conv_real(x_imag) + self.conv_imag(x_real)
        x_real_out = x_real_out.unsqueeze(1)
        x_imag_out = x_imag_out.unsqueeze(1)
        return torch.cat((x_real_out, x_imag_out), dim=1)

class ComplexReLU(nn.Module):
    def forward(self, x):
        x_real = x[:,0]
        x_imag = x[:,1]

        x_real = F.relu(x_real, inplace=True)
        x_real_out = x_real.unsqueeze(1)
        x_imag_out = x_imag.unsqueeze(1)
        return torch.cat((x_real_out, x_imag_out), dim=1)

class ComplexAbs(nn.Module):
    def forward(self, x):
        x_real = x[:,0]
        x_imag = x[:,1]
        return torch.sqrt(x_real**2 * x_imag**2)

class ComplexMLD(nn.Module):
    def __init__(self):
        super().__init__()
        self.layers = nn.Sequential(
            ComplexConv2d(1,4,3, padding=1),
            ComplexReLU(),
            ComplexConv2d(4,8,3, padding=1),
            ComplexReLU(),
            ComplexConv2d(8,10,3, padding=1),
            ComplexAbs(),
        )

    def forward(self, x):
        return self.layers(x)

if __name__ == '__main__':

    d_type = "int"
    csv_fname = "cnn.csv"
	bs = 10
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    # x = torch.randn(2,1,10,10) - using random data
    df = pd.read_csv(csv_fname)                                                              # file containing x,y data
    x = df['x']                                                                              # x axis only
    if d_type == "int": 
        x = torch.tensor(x.values, dtype=torch.int)		
    else:
        x = torch.tensor(x.values, dtype=torch.float32)
    model = ComplexMLD().to(device)                                                          # create model
    opt = torch.optim.Adam(model.parameters())                                               # create optimizer

    for i in range(bs):
        y_ = model.forward(x)
        loss = torch.mean(y_)
        opt.zero_grad()
        loss.backward()
        opt.step()
        print(i, loss.item())
