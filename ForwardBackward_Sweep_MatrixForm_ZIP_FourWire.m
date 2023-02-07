function [Voltage_Node,Current_Node,Iterations]=ForwardBackward_Sweep_MatrixForm_ZIP_FourWire(Grid_Impedance_Z,ApparentPower_Load,Load_Current,ApparentPower_Node,Voltage_Node_Nominal,Voltage_Node_Init,epsilon,itermax)
%--------------------------------------------------------------------------

%##########################################################################
%Forward Backward Sweep algorithm for Radial Distribution System
%
% This function describes a Forward Backward Sweep algorithm for three 
% phase unbalanced power flow. 
%
% It is suitable for radial systems with three standard loads
% 1. Constant power, constant impedance, constant current but in
% in wye connection only. But this is the standard case for low voltage grids.
%
% Markus Bell 08.08.2016
%##########################################################################


%----------------------------------------------------------------------
%Initializing Voltage and Current matrices
l = 0; %Itermax indes
Voltage_Node=Voltage_Node_Init; %Initial voltage is always nominal voltage
Voltage_Node_old=Voltage_Node_Init;
Delta_Busvoltage=Voltage_Node_Init; 
Current_Node = zeros(size(Voltage_Node_Init,1)-3,1);
%----------------------------------------------------------------------

%----------------------------------------------------------------------
%Here starts the sweep
   while(max(abs(Delta_Busvoltage))>epsilon && l<itermax) 
       % Increasde iteration index
       l=l+1;

       %---------------------------------------------------------------
       % Forward-Sweep
       %---------------------------------------------------------------
       %
       % Get momentary current injections based on the new voltages 
       % Updates at all buses except the slack bus 
       for j=1:size(Grid_Impedance_Z,2)
            Current_Node(j) = conj(ApparentPower_Node(j))/...
                                conj(Voltage_Node(3+j))+...
                                conj(ApparentPower_Load(j))*Voltage_Node(3+j)/(Voltage_Node_Nominal(3+j))^2+...
                                Load_Current(j);
       end
       %
       %---------------------------------------------------------------
       
       %---------------------------------------------------------------
       % Backward Sweep
       %---------------------------------------------------------------

       % Calculate the new voltage based on slack bus and voltage drop
       Voltage_Node(4:end) = Voltage_Node_Nominal(4:end)+...
                                Grid_Impedance_Z*Current_Node;

       % Calculat the voltage change for the break statement (are we yet
       % converged)
       Delta_Busvoltage = Voltage_Node-Voltage_Node_old; 
       Voltage_Node_old = Voltage_Node;
       %
       %--------------------------------------------------------------- 
   end
%Here Ends the sweep
%----------------------------------------------------------------------

%Determine the needed iterations
Iterations = l;

end

