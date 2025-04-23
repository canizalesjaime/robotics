import torch
from torch import nn
from sklearn.datasets import make_moons
from sklearn.model_selection import train_test_split
import pandas as pd
import matplotlib.pyplot as plt
from helper_functions import plot_predictions, plot_decision_boundary
import numpy as np


def accuracy_fn(y_true, y_pred):
    correct = torch.eq(y_true, y_pred).sum().item() # torch.eq() calculates where two tensors are equal
    acc = (correct / len(y_pred)) * 100 
    return acc


N = 100 # number of points per class
D = 2 # dimensionality
K = 3 # number of classes
X = np.zeros((N*K,D)) # data matrix (each row = single example)
y = np.zeros(N*K, dtype='uint8') # class labels
for j in range(K):
  ix = range(N*j,N*(j+1))
  r = np.linspace(0.0,1,N) # radius
  t = np.linspace(j*4,(j+1)*4,N) + np.random.randn(N)*0.2 # theta
  X[ix] = np.c_[r*np.sin(t), r*np.cos(t)]
  y[ix] = j
# lets visualize the data
plt.scatter(X[:, 0], X[:, 1], c=y, s=40, cmap=plt.cm.Spectral)
plt.savefig("plot_spiral_1.png")

device = "cuda" if torch.cuda.is_available() else "cpu"
X = torch.from_numpy(X).type(torch.float)
y = torch.from_numpy(y).type(torch.LongTensor)

X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2,
                                                    random_state=42)


class SpiralNet(nn.Module):
    def __init__(self,input_features,output_features,hidden_units=8):
        super().__init__()
        self.model= nn.Sequential(
            nn.Linear(in_features=input_features, out_features=hidden_units),
            nn.ReLU(),
            nn.Linear(in_features=hidden_units, out_features=hidden_units),
            nn.ReLU(),
            nn.Linear(in_features=hidden_units, out_features=output_features)
        )

    def forward(self,x):
        return self.model(x)



# print(X)
# print(y)
spiral_net=SpiralNet(D,K).to(device)
epochs = 100000

loss_fn=nn.CrossEntropyLoss()
optimizer=torch .optim.SGD(params=spiral_net.parameters(),lr=.01)

for epoch in range(epochs):
    spiral_net.train()
    logits_train=spiral_net(X_train).squeeze()
    preds_train = torch.softmax(logits_train, dim=1).argmax(dim=1)
    #print(logits_train)
    #print(y_train)
    train_loss=loss_fn(logits_train,y_train)
    acc_train=accuracy_fn(y_train,preds_train)
    optimizer.zero_grad()
    train_loss.backward()
    optimizer.step()

    #test
    spiral_net.eval()
    with torch.inference_mode():
        logits_test=spiral_net(X_test).squeeze()
        preds_test = torch.softmax(logits_test, dim=1).argmax(dim=1)
        test_loss=loss_fn(logits_test,y_test)
        acc_test=accuracy_fn(y_test,preds_test)

    if epoch % 10 == 0:
        print(f"Epoch: {epoch} | Loss: {train_loss:.5f}, Accuracy: {acc_train:.2f}% | Test loss: {test_loss:.5f}, Test acc: {acc_test:.2f}%")

# Plot decision boundaries for training and test sets
plt.figure(figsize=(12, 6))
plt.subplot(1, 2, 1)
plt.title("Train")
plot_decision_boundary(spiral_net, X_train, y_train)
plt.subplot(1, 2, 2)
plt.title("Test")
plot_decision_boundary(spiral_net, X_test, y_test)
plt.savefig("plot_spiral_2.png")


