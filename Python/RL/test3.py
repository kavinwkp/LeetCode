import torch.optim
from PIL import Image
from torch import nn
from torch.nn import Sequential, Conv2d, MaxPool2d, Flatten, Linear
from torch.utils.data import DataLoader
from torch.utils.tensorboard import SummaryWriter
from torchvision import transforms
from torchvision import datasets
import numpy as np

# image_path = "./images/test.png"
# img = Image.open(image_path)
# print(img.size)
# print(type(img))
# img_array = np.array(img)
# print(img_array.shape)
# tensor_trans = transforms.ToTensor()
# tensor_img = tensor_trans(img)
# print(tensor_img.shape)
# tensor_img2 = tensor_trans(img_array)
# print(tensor_img2.shape)

# trainsets = datasets.MNIST(root="./data", train=True, download=True, transform=transforms.ToTensor())
# testsets = datasets.MNIST(root="./data", train=False, transform=transforms.ToTensor())
# print(trainsets.data.shape)

# test_loader = DataLoader(dataset=testsets, batch_size=64, shuffle=False, num_workers=0, drop_last=True)

# img, target = testsets[0]   # 第一张图片
# print(img.shape)
# print(target)

datasets = datasets.CIFAR10("./data", train=False, transform=transforms.ToTensor(), download=True)
data_loader = DataLoader(datasets, batch_size=1)
print(datasets.data.shape)


class RNN(nn.Module):
    def __init__(self):
        super(RNN, self).__init__()
        self.model1 = Sequential(
            Conv2d(3, 32, kernel_size=(5, 5), padding=(2, 2)),
            MaxPool2d(2),
            Conv2d(32, 32, kernel_size=(5, 5), padding=(2, 2)),
            MaxPool2d(2),
            Conv2d(32, 64, kernel_size=(5, 5), padding=(2, 2)),
            MaxPool2d(2),
            Flatten(),
            Linear(1024, 64),
            Linear(64, 10)
        )

    def forward(self, x):
        x = self.model1(x)
        return x

rnn = RNN()
loss = nn.CrossEntropyLoss()
optim = torch.optim.SGD(rnn.parameters(), lr=0.01)

for epoch in range(2):
    running_loss = 0.0
    for data in data_loader:
        imgs, targets = data
        outputs = rnn(imgs)
        res_loss = loss(outputs, targets)
        optim.zero_grad()
        res_loss.backward()
        optim.step()
        running_loss += res_loss
        # print(res_loss)
        # break
    print(running_loss)

# writer = SummaryWriter("logs")
#
# for epoch in range(2):
#     step = 0
#     for data in test_loader:
#         imgs, targets = data
#         # print(imgs.shape)
#         # print(targets)
#         writer.add_images("epoch: {}".format(epoch), imgs, step)
#         step += 1
#
# writer.close()
