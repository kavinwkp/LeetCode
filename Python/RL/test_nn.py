import torch
from torch import nn


# class rnn(nn.Module):
#     def __init__(self):
#         super().__init__()
#
#     def forward(self, input):
#         output = input.mm(input.t())
#         return output
#
# r = rnn()
# x = torch.tensor([[3, 4]])
# out = r.forward(x)
# print(out)

# input = torch.arange(25)
# print(input)
# input = torch.reshape(input, (5, -1))
# print(input)

a = torch.tensor([1, 2, 3.])
b = torch.tensor([1, 4, 3.])

a = torch.reshape(a, (1, 1, 1, -1))
b = torch.reshape(b, (1, 1, 1, -1))

loss = nn.MSELoss()
result = loss(a, b)
print(result)

# Example of target with class indices
loss = nn.CrossEntropyLoss()
input = torch.randn(2, 3, requires_grad=True)
print(input)
target = torch.empty(2, dtype=torch.long).random_(3)
print(target)
output = loss(input, target)
print(output)
output.backward()

# Example of target with class probabilities
# input = torch.randn(3, 5, requires_grad=True)
# target = torch.randn(3, 5).softmax(dim=1)
# output = loss(input, target)
# output.backward()