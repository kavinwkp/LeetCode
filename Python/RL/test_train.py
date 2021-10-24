import torch
import torch.optim
from torch import nn
from torch.nn import Sequential, Conv2d, MaxPool2d, Flatten, Linear
from torch.utils.data import DataLoader
from torch.utils.tensorboard import SummaryWriter
from torchvision import transforms
from torchvision import datasets
from CNN import *
import time


writer = SummaryWriter("logs")


train_data = datasets.CIFAR10("./data", train=True, transform=transforms.ToTensor(), download=True)
test_data = datasets.CIFAR10("./data", train=False, transform=transforms.ToTensor(), download=True)

train_data_size = len(train_data)
test_data_size = len(test_data)
# print("train: {}".format(train_data_size))  # 50000
# print("test: {}".format(test_data_size))    # 10000
# print(train_data.data.shape)                # (50000, 32, 32, 3)

train_data_loader = DataLoader(train_data, batch_size=64)
test_data_loader = DataLoader(test_data, batch_size=64)

cnn = CNN()     # 创建网络模型
if torch.cuda.is_available():
    cnn = cnn.cuda()

loss_fn = nn.CrossEntropyLoss()
if torch.cuda.is_available():
    loss_fn = loss_fn.cuda()

learning_rate=  1e-2
optimizer = torch.optim.SGD(cnn.parameters(), lr=learning_rate)

train_step = 0
test_step = 0
epoch = 3

start_time = time.time()
for i in range(epoch):
    print("-----第{}轮训练------".format(i + 1))

    for data in train_data_loader:
        imgs, targets = data
        if torch.cuda.is_available():
            imgs = imgs.cuda()
            targets = targets.cuda()
        output = cnn(imgs)
        loss =  loss_fn(output, targets)

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        train_step += 1
        if (train_step % 100 == 0):
            print("train step: {}, loss: {}".format(train_step, loss.item()))
            writer.add_scalar("train_loss", loss.item(), train_step)

    # 测试阶段，不更新梯度
    total_test_loss = 0
    total_accuracy = 0
    with torch.no_grad():
        for data in test_data_loader:
            imgs, targets = data
            if torch.cuda.is_available():
                imgs = imgs.cuda()
                targets = targets.cuda()
            output = cnn(imgs)
            loss = loss_fn(output, targets)
            total_test_loss += loss.item()
            accuracy = (output.argmax(1) == targets).sum()
            total_accuracy += accuracy
    test_step += 1
    print("整体测试集的损失: {}".format(total_test_loss))
    print("整体测试集的正确率: {}".format(total_accuracy / test_data_size))
    writer.add_scalar("Test_loss", total_test_loss, test_step)
    writer.add_scalar("Test_accuracy", total_accuracy / test_data_size, test_step)


writer.close()

end_time = time.time()
print("time: ", end_time - start_time)
