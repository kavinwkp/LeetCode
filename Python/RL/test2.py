import torch
from torch.utils.tensorboard import SummaryWriter
import numpy as np

writer = SummaryWriter("logs")
for i in range(100):
    writer.add_scalar("y=x", i * i - 1, i)

writer.close()


a = torch.tensor([1, 4, 7], dtype=torch.float)
print(torch.mean(a))
print(torch.var(a))
print(torch.std(a))
print(torch.median(a))
print(torch.sum(a))
print(torch.prod(a))
print(torch.min(a))
print(torch.max(a))
print(torch.argmax(a))  # 最大值的下标
print("\n")

print(torch.manual_seed(233))

a1 = torch.tensor([[1, 2, 3], [8, 9, 10]])
a2 = torch.tensor([[4, 5, 6],
                   [10, 4, 2]])
print(a1.shape)
print(a2.shape)
print(torch.cat((a1, a2), dim=0))  # 按行连接
print(torch.cat((a1, a2), dim=1))  # 按行连接
print(torch.stack([a1, a2], dim=0))  # stack要求大小必须相同
print(torch.stack([a1, a2], dim=1))
print(torch.stack([a1, a2], dim=2))
# print(torch.where(a1 > 1))
print("\n")

a = torch.linspace(1, 12, steps=12).view(3, 4)
print(a)
b = torch.index_select(a, 0, torch.tensor([1, 2]))
print(b)
print(torch.index_select(a, 1, torch.tensor([1, 2])))
print(torch.Tensor([3.4]))
mask = a.ge(7)  # >= 7
print(mask)
print(torch.masked_select(a, mask))  # 输出是一个向量
print(torch.take(a, torch.tensor([0, 3, 6])))

a = torch.tensor([[1, 2, 3]])
b = torch.tensor([[4, 5, 6], [7, 8, 9]])
c = torch.cat([a, b], dim=0)
print(c)
d = torch.chunk(c, 3, dim=0)
print(d)
print(len(d))
print(d[0].shape)

e = torch.split(c, 1, dim=1)
print(e)
print(e[0].shape)
f = torch.split(c, [1, 2], dim=0)
print(f)
print(f[1].shape)

print("\n")

print(a)
print(a.t_())
print(a)

a = torch.tensor([[1], [2]])
print(a)
a1 = torch.squeeze(a)
print(a1)

b = torch.tensor([1, 2, 3, 4])
print(b)
b1 = torch.unsqueeze(b, 1)
print(b1)

a = torch.tensor([[1, 2, 3], [4, 5, 6]])
print(a)
print(torch.flip(a, [0]))
