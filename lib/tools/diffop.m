%
% Author: Jan Lellmann, J.Lellmann@damtp.cam.ac.uk
% Date: 18/11/2013
%
% creates 1-dimensional sparse difference operator matrix for
% vectors with length n
% boundaries can be a string or a cell array of 2 strings for left and
% right boundary conditions
% normbnd is optional returns an upper bound for the operator's norm if
% available (and +inf if not).
% NOTE: A grid spacing of _2_ is assumed!
%
function [result,resultbound] = diffop(n, scheme, boundaries)
    result = [];
    normbound = +inf;
    
    if (n < 1)
        error('n must be positive');
    end
    
    if (strcmp(class(boundaries),'char'))                                 % check for 'cond'
        boundaries = {boundaries,boundaries};
    elseif (strcmp(class(boundaries),'cell') && (numel(boundaries) == 1)) % check for {'cond'}
        boundaries = {boundaries{1},boundaries{1}};
    elseif (~strcmp(class(boundaries),'cell') || ~(numel(boundaries) == 2))
        % does not check class of cell array elements
        error('invalid boundary condition specification - must be string or 2-element cell array of strings');
    end
    
    if (strcmp(scheme,'forward'))
        %TODO improve estimate for neumann & dirichlet conditions (can
        %probably be computed explicitly)
        %TODO look up proof for this (experimentally verified using svd(...); for periodic conditions:
        %2.0 is exactly reached.
        if (strcmp(boundaries(2),'neumann'))
            result = spdiags([-ones(n-1,1) ones(n-1,1); 0 1],0:1,n,n);            
            normbound = 2;
        elseif (strcmp(boundaries(2),'dirichlet'))
            result = spdiags([-ones(n,1) ones(n,1)],0:1,n,n);
            normbound = 2;
        elseif (strcmp(boundaries(2),'periodic'))
            result = spdiags([-ones(n,1) ones(n,1)],0:1,n,n);
            result(n,1) = result(n,1) + 1; % sum catches the extreme case n = 1
            normbound = 2;
        elseif (strcmp(boundaries(2),'second-order'))
            result = spdiags([-ones(n-1,1) ones(n-1,1); 0 1],0:1,n,n);            
            result(end,:) = result(end-1,:);
            normbound = +inf; % not implemented yet
        else
            error('unsupported boundary condition for this scheme');
        end            
    elseif (strcmp(scheme,'backward'))
        if (strcmp(boundaries(1),'neumann'))
            result = spdiags([-1 0; -ones(n-1,1) ones(n-1,1)],-1:0,n,n);
            normbound = 2;
        elseif (strcmp(boundaries(1),'dirichlet'))
            result = spdiags([-ones(n,1) ones(n,1)],-1:0,n,n);
            normbound = 2;
        elseif (strcmp(boundaries(1),'periodic'))
            result = spdiags([-ones(n,1) ones(n,1)],-1:0,n,n);
            result(1,n) = result(1,n) - 1; % sum catches the extreme case n = 1
            normbound = 2;
        elseif (strcmp(boundaries(2),'second-order'))
            result = spdiags([-1 0; -ones(n-1,1) ones(n-1,1)],-1:0,n,n);
            result(1,:) = result(2,:);
            normbound = +inf; % not implemented yet            
        else
            error('unsupported boundary condition for this scheme');
        end
%{        
    % central differences are not completely thought over - i.e. boundary
    % condition handling: should the first line be
    % [0 0 0...] (->currently implemented; boundary conditions formulated with central differences) or
    % [1 -1 0...] (->boundary conditinos formulated by copying boundary elements)
    %FIXME DOES NOT SUPPORT DIFFERENT LEFT AND RIGHT BOUNDARY CONDITIONS
    %YET
    elseif (strcmp(scheme,'central'))
        % central differences are pretty useless for our case
        % (->oscillations, awkward conj_op * op), but are supported for
        % completeness. Note the 
        % different meaning than
        if (strcmp(boundary,'neumann'))
            result = spdiags([-ones(n,1),zeros(n,1),ones(n,1)],-1:1,n,n);
            result(1,:) = 0;
            result(n,:) = 0;
        elseif (strcmp(boundary,'dirichlet'))
            result = spdiags([-ones(n,1),zeros(n,1),ones(n,1)],-1:1,n,n);
        elseif (strcmp(boundary,'periodic'))
            result = spdiags([-ones(n,1),zeros(n,1),ones(n,1)],-1:1,n,n);
            result(1,n) = result(1,n) - 1; % works also for n = 1
            result(n,1) = result(n,1) + 1;
        else
            error('unsupported boundary condition for this scheme');
        end
%}
    else
        error('unsupported scheme');
    end    
    
    if (nargout > 1)
        resultbound = normbound;
    end
end

%UNTITLED1 Summary of this function goes here
%   Detailed explanation goes here
