function [BC_Struct] = getGlobalBC()
    global boundary_conditions;
    BC_Struct = boundary_conditions;
end