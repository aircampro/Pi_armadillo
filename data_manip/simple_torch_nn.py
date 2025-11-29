#
# simple linear network using torch 3 -> 2 -> 1 or X->Y->Z
#
import torch.nn as nn
import torch.nn.functional as F
import torch
import sys

class Net(nn.Module):
    def __init__(self,X,Y,Z):
        super(Net, self).__init__()
        self.fc1 = nn.Linear(X, Y)
        self.fc2 = nn.Linear(Y, Z)
    def forward(self, x):
        x = self.fc1(x)
        x = F.relu(x)
        x = self.fc2(x)
        return x

def main():
    # take dimensions for the command line
    if len(sys.argv[0]) >= 3:
        X=int(sys.argv[0])
        Y=int(sys.argv[0])
        Z=int(sys.argv[0])
    else:
        X=3
        Y=2
        Z=1

    torch.manual_seed(1)
    # create input tensor for nodes
    tt_array=[]
    for i in range(0,X):
        tt_array.append(i+1)
    x = torch.tensor([tt_array], dtype=torch.float32)
    # define input and output linear networks
    fc1 = nn.Linear(X, Y)
    fc2 = nn.Linear(Y, Z)
    # input 
    u1 = fc1(x)
    # Nonlinear transform ReLU function as the Activation function
    z1 = F.relu(u)
    # output
    y = fc2(z1)
    print(f"output result y {y}")
    t = torch.tensor([[1]], dtype=torch.float32)
    loss = F.mse_loss(t, y)
    print(f"loss {loss}")

    # same with class
    nnet=Net(X,Y,Z)
    print(f"output result {nnet}")
	
if __name__ == "__main__":
    main()

