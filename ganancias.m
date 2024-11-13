ts=6.0;

z = 0.7;

wn = 3/(ts*z)

p2=2*z*wn
p3=wn*wn
p1=5.0

k0 = p1*p3
k1 = p3+p1*p2
k2 = p1+p2
k=k0/k2

raices = roots([1, k2, k1, k0])

T=70;
f=1/T;
w=2*pi*f