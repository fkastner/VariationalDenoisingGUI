%
% Author: Jan Lellmann, J.Lellmann@damtp.cam.ac.uk
% Date: 18/11/2013
%
function diffop_demo

    % one-dimensional data, so grad, div are just the usual derivatives

    % how to do it

    G = full(diffop(5,'forward','neumann')) % <grad u,n> = 0 boundary conditions
    
    D = -G' % <v,n>=0 boundary conditions, D is adjoint to G
    
    D * G % Laplace (1D -> second derivatives) for Neumann boundary conditions
    
    
    % how not to do it
    
    G = full(diffop(5,'forward','neumann'))
    
    D = full(diffop(5,'backward','neumann'))
    %D = full(diffop(5,'forward','neumann')) % variant
    
    D * G % incorrect boundary conditions
end
