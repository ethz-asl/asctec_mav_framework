load 'rtw_arm7_config'

%% generate symbolic expression for state prediction
ekf_zero_order_propagation_sym

%% generate matlab code from symbolic expression
matlabFunction(x_new, 'file', 'autogen_ekf_propagation', 'vars', {x_old, acc, om, dt})
add_eml_compliance('autogen_ekf_propagation.m')

%% generate c code
vec_3_props = single(zeros(3,1));
vec_4_props = single(zeros(4,1)); 
vec_16_props = single(zeros(20,1)); 
scalar_props = single(0);

emlc  -T RTW -c  -v -s hw_impl -o pos_ekf -d d_ekf_src/ekf... 
    autogen_ekf_propagation -eg {vec_16_props, vec_3_props, vec_3_props, scalar_props}