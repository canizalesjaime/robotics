import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import requests
from pathlib import Path 


plt.plot([1, 2, 3, 4], [1, 4, 9, 16])
plt.xlabel("X-axis")
plt.ylabel("Y-axis")
plt.title("Test Plot")
plt.show()


# Download helper functions from Learn PyTorch repo (if not already downloaded)
if Path("helper_functions.py").is_file():
  print("helper_functions.py already exists, skipping download")
else:
  print("Downloading helper_functions.py")
  request = requests.get("https://raw.githubusercontent.com/mrdbourke/pytorch-deep-learning/main/helper_functions.py")
  with open("helper_functions.py", "wb") as f:
    f.write(request.content)
