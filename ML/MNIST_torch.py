#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# MNIST using pytorch
#
import torch
from torch.utils.data import DataLoader
from torchvision import transforms
from torchvision.datasets import MNIST

if __name__ == '__main__':
    # GPU or CPU                                                                
    device = 'cuda' if torch.cuda.is_available() else 'cpu'

    # model                                                                         
    model = cnn().to(device)
    opt = torch.optim.Adam(model.parameters())

    # dataset                                                                    
    bs = 128                                                                                                   # batch size                                                                  
    transform = transforms.Compose([transforms.ToTensor(), transforms.Normalize((0.5,), (0.5,))])

    trainset = MNIST(root='./data', train=True, download=True, transform=transform)
    trainloader = DataLoader(trainset, batch_size=bs, shuffle=True)

    testset = MNIST(root='./data', train=False, download=True, transform=transform)
    testloader = DataLoader(testset, batch_size=bs, shuffle=False)

    # training                                                                             
    model = model.train()
    for iepoch in range(3):
        for iiter, (x, y) in enumerate(trainloader, 0):                                             
            x = x.to(device)
            y = torch.eye(10)[y].to(device)                                                     
            y_ = model.forward(x)                                    # y_.shape = (bs, 84)                                   
            # loss: cross-entropy                                                          
            eps = 1e-7
            loss = -torch.mean(y*torch.log(y_+eps))
            opt.zero_grad() 
            loss.backward() )
            opt.step() 
            if iiter%100==0:
                print('%03d epoch, %05d, loss=%.5f' % (iepoch, iiter, loss.item()))

    # test
    total, tp = 0, 0
    model = model.eval()
    for (x, label) in testloader:                                                           
        x = x.to(device)                                                           
        y_ = model.forward(x)
        label_ = y_.argmax(1).to('cpu')                                                       
        total += label.shape[0]
        tp += (label_==label).sum().item()
    acc = tp/total
    print('test accuracy = %.3f' % acc)