#!/usr/bin/env python3
# Called by tools/ncnn/make_ncnn.sh: fix_ncnn.py SIM.onnx IN.param IN.bin OUT.param OUT.bin
# Fix pnnx output for supercombo v0.9.7:
# 1) ReduceSum axes (opset13 input) lost -> set axes=[0] on sum_237/sum_238
# 2) Gather(features_buffer, axis1, [-5..-40]) wrongly became a 1-row Crop -> drop Crop+Squeeze,
#    in6 becomes the host-gathered [8,512] rows (99-5k, k=1..8) fed straight into cat_2
# 3) 2-D Gemm (slow on turnip: ~1 s total) -> InnerProduct; strip 4-byte tag before each Gemm bias
# 4) fp16 range: the vision feature L2 normalisation (mul_158 = x*x, sum_237, sqrt_159, div_160) has a
#    sum of squares up to ~1.6e5 on real frames, above fp16's 65504; with fp16-packed storage it
#    saturates and the normalised feature (which also feeds the recurrent features_buffer) comes out
#    up to 1.5x too small (lane lines off by metres on some frames). Scale one factor of the square by
#    2^-8 and the norm back by 2^4 (exact in fp32, identical result): +2 layers, +2 blobs.
import sys, re, numpy as np, onnx
from onnx import numpy_helper as nh
onx, pin, bin_, pout, bout = sys.argv[1:6]
L = open(pin).read().splitlines()
out=[]
for l in L:
    f=l.split()
    if f and f[0]=='Reduction' and f[1] in ('sum_237','sum_238'): l=l.replace(' -23303=0 ',' -23303=1,0 ')
    if f and f[0]=='Crop' and f[1]=='select_0': continue
    if f and f[0]=='Squeeze' and f[1]=='squeeze_235': continue
    if f and f[0]=='Concat' and f[1]=='cat_2': l=re.sub(r' 459 455 ',' 459 in6 ',l)
    if f and f[1:2]==['mul_158']:
        assert f[4:7]==['444','445','447'], l
        out.append('%-24s %-24s 1 1 444 444s 0=2 1=1 2=0.00390625'%('BinaryOp','fp16_prescale'))
        l=l.replace(' 444 445 447 ',' 444s 445 447 ')
    if f and f[1:2]==['clamp_231']:
        assert f[4:6]==['449','450'], l
        out.append('%-24s %-24s 1 1 449 449s 0=2 1=1 2=16'%('BinaryOp','fp16_postscale'))
        l=l.replace(' 449 450 ',' 449s 450 ')
    if f and f[0]=='Gemm':
        kv=dict(x.split('=') for x in f[6:]); n,k=int(kv['8']),int(kv['9'])
        l='%-24s %-24s %s %s %s %s 0=%d 1=1 2=%d'%('InnerProduct',f[1],f[2],f[3],f[4],f[5],n,n*k)
    out.append(l)
nl,nb=map(int,out[1].split()); out[1]='%d %d'%(nl-2+2,nb-2+2)  # -Crop -Squeeze (3), +2 scale layers (4)
open(pout,'w').write('\n'.join(out)+'\n')
m=onnx.load(onx); init={i.name:nh.to_array(i) for i in m.graph.initializer}
b=bytearray(open(bin_,'rb').read()); pos=[]
for n in m.graph.node:
    if n.op_type=='Gemm':
        at={a.name:a.i for a in n.attribute}
        if at.get('transB',0)!=0: continue
        W=init[n.input[1]]; bias=init[n.input[2]]; K,N=W.shape
        i=b.find(np.ascontiguousarray(W.T[0,:16]).tobytes()); bp=i+N*K*4
        assert i>0 and b[bp:bp+4]==b'\0\0\0\0' and b[bp+4:bp+4+N*4]==bias.tobytes(), n.name
        pos.append(bp)
assert len(pos)==5
for p in sorted(pos,reverse=True): del b[p:p+4]
open(bout,'wb').write(b); print('fixed', pout, bout)
