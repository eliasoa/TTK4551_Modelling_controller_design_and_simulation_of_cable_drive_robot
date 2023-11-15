function f = CDPR_Control_Allocation(tau, A_transposed)

A_PseudoInv = A_transposed'/(A_transposed*A_transposed');

f = A_PseudoInv*tau;
end

