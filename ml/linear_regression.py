import torch
from torch import nn
import matplotlib.pyplot as plt
from pathlib import Path

class LinearRegressionModel(nn.Module):
  def __init__(self):
    super().__init__()
    self.linear_layer=nn.Linear(in_features=1,out_features=1)

  def forward(self, x):
    return self.linear_layer(x)

device = "cuda" if torch.cuda.is_available() else "cpu"
weight , bias = .3,.9
X=torch.arange(0,1,.01).unsqueeze(dim=1)
y=weight*X+bias

train_split=int(len(X)*.8)
X_train, y_train = X[:train_split].to(device), y[:train_split].to(device)
X_test, y_test = X[train_split:].to(device) , y[train_split:].to(device)

torch.manual_seed(42)
m=LinearRegressionModel().to(device)
loss=nn.L1Loss()
optimizer=torch.optim.SGD(params=m.parameters(),lr=.01)
for epoch in range(300):
  m.train()
  output=m(X_train)
  train_loss=loss(output,y_train)
  optimizer.zero_grad()
  train_loss.backward()
  optimizer.step()

  if epoch%20==0:
    m.eval()
    with torch.inference_mode():
      test_preds=m(X_test)
      test_loss=loss(test_preds,y_test)
    #print(train_loss, test_loss)

# m.eval()
# with torch.inference_mode():
#   test_preds=m(X_test)
#   test_loss=loss(test_preds,y_test)
print(test_preds)
print(m.state_dict())
plt.scatter(X, y, s=4,c='g',label='ground_truths')
plt.scatter(X_test.cpu(), test_preds.cpu(), s=4,c='r',label='test')
#plt.show()
plt.savefig("plot.png")  # Saves the plot instead of showing it

a=Path("models")
a.mkdir(parents=True, exist_ok=True)
b=Path("aaa.pth")
a_b=a/b
torch.save(obj=m.state_dict(),f=a_b)

loaded_model = LinearRegressionModel().to(device)
loaded_model.load_state_dict(torch.load(f=a_b))

loaded_model.eval()
with torch.inference_mode():
  out=loaded_model(X_test)
  print(out)
