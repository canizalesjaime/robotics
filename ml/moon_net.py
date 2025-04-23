import torch
from torch import nn
from sklearn.datasets import make_moons
from sklearn.model_selection import train_test_split
import pandas as pd
import matplotlib.pyplot as plt
from helper_functions import plot_predictions, plot_decision_boundary

def accuracy_fn(y_true, y_pred):
    correct = torch.eq(y_true, y_pred).sum().item() # torch.eq() calculates where two tensors are equal
    acc = (correct / len(y_pred)) * 100 
    return acc

class MoonNet(nn.Module):
    def __init__(self, input_features, output_features, hidden_units=8):
        super().__init__()
        
        self.model = nn.Sequential(
            nn.Linear(in_features=input_features, out_features=hidden_units),
            nn.ReLU(), 
            nn.Linear(in_features=hidden_units, out_features=hidden_units),
            nn.ReLU(),
            nn.Linear(in_features=hidden_units, out_features=output_features)
        )

    def forward(self,x):
        return self.model(x)


device = "cuda" if torch.cuda.is_available() else "cpu"
X, y = make_moons(n_samples=1000, noise=0.2, random_state=42)
X = torch.from_numpy(X).type(torch.float)
y = torch.from_numpy(y).type(torch.float)
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2,
                                                    random_state=42)
plt.scatter(x=X[:, 0], y=X[:, 1], c=y, cmap=plt.cm.RdYlBu)
plt.savefig("plot_moon_1.png")

moon_model = MoonNet(input_features=2,output_features=1).to(device)
loss_fn = nn.BCEWithLogitsLoss()
optimizer = torch.optim.SGD(params=moon_model.parameters(), lr=0.1)


epochs = 1000
for epoch in range(epochs):
    moon_model.train()
    logits_train=moon_model(X_train.to(device)).squeeze()
    y_preds_train=torch.round(torch.sigmoid(logits_train))
    train_loss=loss_fn(logits_train,y_train)
    acc_train=accuracy_fn(y_train,y_preds_train)
    optimizer.zero_grad()
    train_loss.backward()     
    optimizer.step()

    # test 
    with torch.inference_mode():
        moon_model.eval()
        logits_test=moon_model(X_test.to(device)).squeeze()
        y_preds_test=torch.round(torch.sigmoid(logits_test))
        test_loss=loss_fn(logits_test,y_test)
        acc_test=accuracy_fn(y_test, y_preds_test)
        
    if epoch % 10 == 0:
        print(f"Epoch: {epoch} | Loss: {train_loss:.5f}, Accuracy: {acc_train:.2f}% | Test loss: {test_loss:.5f}, Test acc: {acc_test:.2f}%")


# Plot decision boundaries for training and test sets
plt.figure(figsize=(12, 6))
plt.subplot(1, 2, 1)
plt.title("Train")
plot_decision_boundary(moon_model, X_train, y_train)
plt.subplot(1, 2, 2)
plt.title("Test")
plot_decision_boundary(moon_model, X_test, y_test)
plt.savefig("plot_moon_2.png")

