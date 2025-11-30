#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Linear regression/network using torch
#
import numpy as np
from matplotlib import pyplot as plt
import pandas as pd
import torch
from torch import nn
from torch import optim
import sys

# define mode as test with random for regression or from a csv file
mode="from_csv"
# the file is from the command argument or defaulted as "xy_data.csv"
if mode == "from_csv":
    if len(sys.argv[0]) >= 1:
        csv_fname = str(sys.argv[1])
    else:
        csv_fname = "xy_data.csv"
# choose model linear_reg or linesr_net
model_name="linear_reg"
# category for classification or reg for regression
y_type="reg"                      

# define a linear regression
#                                                       
class LinearRegression(nn.Module):
    def __init__(self):
        super().__init__()
        self.layer = nn.Linear(1, 1, bias=True)

    def forward(self, x):
        y = self.layer(x)
        return y

# define a linear network
# input size number of inputs (sensor measurements)
# output size number of categories (results)
#
class Net(nn.Module):

    def __init__(self, input_size=10, hidden_size=5, output_size=3, batch_size=10):
        super(Net, self).__init__()
        self.fc1 = nn.Linear(input_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, output_size)
        self.batch_size = batch_size

    def forward(self, x):
        x = self.fc1(x)
        x = F.relu(x)
        x = self.fc2(x)
        return x
        
if __name__ == '__main__':

    # GPU/CPU                                                      
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    bs = 10                                                                                     # batch_size 

    if not model_name.find("linear_reg") == -1:                                                  
        model = LinearRegression().to(device)
        opt = optim.SGD(model.parameters(), lr=0.01)                                            # model optimizer 
        niter = 1000
    elif model_name.find("linear_net") == -1:
        model = Net()                                                                           # using our default net input_size=10, hidden_size=5, output_size=3
        optimizer = optim.SGD(model.parameters(), lr=0.01)
        criterion = nn.CrossEntropyLoss()
        sum_loss = 0.0 
        niter = 5000

    # data                                                                  
    n = 1000
    if mode == "random":                                                                         # for linear regression test only
        x = torch.rand(n)*2-1                                                                    # create randm data
        a, b = 2.0, -10.0                                                                        # weight & bias                                           
        y = a*x+b                                                     
        x = x + torch.randn(n)*0.02                                                              # data + small ammount of noise  
        y = y + a*torch.randn(n)*0.02
    elif mode == "from_csv":                                                                     # gets the data from csv using pandas dataframe
        df = pd.read_csv(csv_fname)                                                              # file containing x,y data
        t = df['y']                                                                              # y for example is time axis
        x = df.drop('y', axis=1)                                                                 # x is everything else in the case of (linear reg) only x otherwise input_size=10
        x = torch.tensor(x.values, dtype=torch.float32)
        if not y_type.find("cat") == -1:
            t = torch.tensor(t.values, dtype=torch.int)
        else:
            t = torch.tensor(t.values, dtype=torch.float32)
        dataset = torch.utils.data.TensorDataset(x, t)
        n_train = int(len(dataset) * 0.6)
        n_val = int(len(dataset) * 0.2)
        n_test = len(dataset) - n_train - n_val
        torch.manual_seed(0)
        train, val, test = torch.utils.data.random_split(dataset, [n_train, n_val, n_test])
        x_t, t_t = torch.utils.data.DataLoader(train, bs, shuffle=True)
        x_train = torch.tensor(x_t, dtype = torch.float32)
        y_train = torch.tensor(t_t, dtype = torch.float32)
        x = x_train
        y = y_train

    # to GPU if it can                                                                   
    x = x.to(device)
    y = y.to(device)                                                       

    losses = []
    for iiter in range(niter):

        # batch data                                                      
        r = np.random.choice(n, bs, replace=False)
        bx = x[r].reshape(-1,1)
        by = y[r].reshape(-1,1)

        if not model_name.find("linear_reg") == -1:
            # forward loss                                                     
            y_ = model.forward(bx)
            loss = torch.mean((y_ - by)**2)

            # 最適化                                                                
            opt.zero_grad()                             # 勾配初期化                                            
            loss.backward()                             # 勾配計算(backward)                                    
            opt.step()                                  # パラメータ更新         
            print('%05d/%05d loss=%.5f' % (iiter, niter, loss.item()))
            losses.append(loss.item())                 
        elif model_name.find("linear_net") == -1:
            # linear net
            optimizer.zero_grad()
            output = model(bx)
            loss = criterion(output, y)
            loss.backward()
            optimizer.step()
            sum_loss += loss.item()
            if i % 1000 == 0:
                print("loss : {0}".format(sum_loss/i))

    if not model_name.find("linear_reg") == -1:                                                           
        a_ = model.layer.weight.detach().to('cpu').numpy()                            # linear regression result   
        b_ = model.layer.bias.detach().to('cpu').numpy()
        print('a=%.3f b=%.3f' % (a_[0] ,b_[0]))
        xnp = x.detach().to('cpu').numpy()                                            # plot            
        ynp = y.detach().to('cpu').numpy()
        plt.scatter(xnp, ynp)
        x = np.linspace(-1,1,100)
        y = a_[0]*x + b_[0]
        plt.plot(x, y, c='r')
        plt.savefig('lr_output.png')
        plt.tight_layout();plt.show() 
    elif not model_name.find("linear_net") == -1: 
        x_t, t_t = torch.utils.data.DataLoader(val, bs, shuffle=True)
        X_val = torch.tensor(x_t, dtype = torch.float32)
        if not y_type.find("cat") == -1:
            y_val = torch.tensor(t_t, dtype = torch.int)                               # category int (discrete value) or regression float32
        else:
            y_val = torch.tensor(t_t, dtype = torch.float32)        
        outputs = model(X_val)
        _, predicted = torch.max(outputs.data, 1)
        y_predicted = predicted.numpy()
        accuracy = 100 * np.sum(predicted.numpy() == y_val) / len(y_predicted)
        print('validation accuracy: {:.1f}%'.format(accuracy))
        x_t, t_t = torch.utils.data.DataLoader(test, bs, shuffle=True)
        X_test = torch.tensor(x_t, dtype = torch.float32)
        if not y_type.find("cat") == -1:
            y_test = torch.tensor(t_t, dtype = torch.int)                               # category int (discrete value) or regression float32
        else:
            y_test = torch.tensor(t_t, dtype = torch.float32)        
        outputs = model(X_test)
        _, predicted = torch.max(outputs.data, 1)
        y_predicted = predicted.numpy()
        accuracy = 100 * np.sum(predicted.numpy() == y_test) / len(y_predicted)
        print('test accuracy: {:.1f}%'.format(accuracy))