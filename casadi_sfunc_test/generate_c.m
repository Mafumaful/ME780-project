% please see https://web.casadi.org/blog/s-function/

import casadi.*

x = MX.sym('x',2);
y = MX.sym('y');

w = dot(x,y*x);
z = sin(x)+y+w;

f = Function('f',{x,y},{w,z});

f.generate('f.c')