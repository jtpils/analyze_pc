#!/usr/bin/python
from PIL import Image

images = map(Image.open, [str(i)+"_plot.png" for i in range(1,10)])
nw=3
nh=3
w = nw*images[0].size[0]
h = nh*images[0].size[1]

result = Image.new("RGBA", (w,h))

x=0
y=0
for i in images:
    result.paste(i, (x%(nw*i.size[0]),i.size[1]*(y/nw)))
    x+=i.size[0]
    y+=1

result.save("hist_1_11.png")

