import torch
import torchvision

vgg16 = torchvision.models.vgg16(pretrained=False)
# print(vgg16)
#
# torch.save(vgg16, "vgg16_model.pth")  # 1: 保存模型结构+参数

# model = torch.load("vgg16_model.pth")   # 1: 加载模型
# print(model)

# torch.save(vgg16.state_dict(), "vgg16_model2.pth")  # 2: 保存模型参数

output = torch.tensor([[1, 2], [3, 4], [4, 6]])
preds = output.argmax(1)
print(preds)
targets = torch.tensor([0, 1, 1])
res = (preds == targets)
print(res)
print(torch.sum(res))

print("\n")
a = torch.randn(2, 3)
print(a)
print(torch.sum(a, dim=0))
print(torch.sum(a, dim=1))
