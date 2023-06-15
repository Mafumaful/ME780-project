% please see https://web.casadi.org/blog/s-function/

cg_options = struct;
cg_options.casadi_real = 'real_T';
cg_options.casadi_int = 'int_T';
cg_options.with_header = true;
cg = CodeGenerator('f',cg_options);
cg.add_include('simstruc.h');
cg.add(f);
cg.generate();