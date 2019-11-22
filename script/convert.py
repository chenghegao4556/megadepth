import torch
import numpy as np
import pytorch_DIW_scratch


model = pytorch_DIW_scratch.pytorch_DIW_scratch
new_dict = {}
dic = torch.load('./best_generalization_net_G.pth')
for name in dic.keys():
    a = str(name)
    b = a.split('.', 1 )[1]
    new_dict[b] = dic[name]
print("start convert")
model.load_state_dict(new_dict)
meganet = torch.jit.trace(model, torch.rand(1,3,384,512))
meganet.save('mega_depth_net.pt')
