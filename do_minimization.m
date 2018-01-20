function [ x, stats ] = do_minimization( img, params, alpha, argmin )
%DO_MINIMIZATION Summary of this function goes here
%   Detailed explanation goes here

% Set default values
if nargin < 3
    if isfield(params, 'alpha')
        alpha = params.alpha;
    else
        alpha = 1;
    end
end
if nargin < 4
    argmin = true;
end

fidelity = params.fidelity;
regularizer = params.regularizer;
s = size(img);
n_ghosts = 1; % Number of ghost cells at each boundary
sg = s + (2*n_ghosts); % Size including ghost cells


cvx_begin quiet
    
    cvx_solver Mosek

    % Set up variables
    variable u(sg)             % including ghost cells
    0 <= u <= 255;
    data = u(1+n_ghosts:end-n_ghosts,...
             1+n_ghosts:end-n_ghosts);  % excluding ghost cells
    

    % Define fidelity term
    switch fidelity
        case 'L1'           % ||u-image||_1
            dataterm = norm( data(:)-img(:) , 1 );

        case 'L2'           % ||u-image||_2
            dataterm = norm( data(:)-img(:) , 2 );

        case 'L2 squared'   % ||u-image||_2^2
            dataterm = sum_square( data(:) - img(:) );

        case 'Residual'     % 0 s.t. ||u-image||_2 <= delta
            dataterm = 0;
            delta = params.delta;
            norm( data(:)-img(:), 2 ) <= 1.1 * delta;

        case 'Bounds'       % 0 s.t. fl <= u <= fu
            dataterm = 0;
            f_lower = params.lowerbounds;
            f_upper = params.upperbounds;
            f_lower(:) <= data(:) <= f_upper(:);
            
        case 'Relaxed Bounds L1'   % ||(fl-u)+||_1 + ||(u-fu)+||_1
            f_lower = params.lowerbounds;
            f_upper = params.upperbounds;
            relaxed_lower = sum(pos(f_lower(:) - data(:)));
            relaxed_upper = sum(pos(data(:) - f_upper(:)));
            dataterm = (relaxed_lower) + (relaxed_upper);
            
        case 'Relaxed Bounds Residual' % 0 s.t. ||(fl-u)+||_1 + ||(u-fu)+||_1 <= ??
            dataterm = 0;
            f_lower = params.lowerbounds;
            f_upper = params.upperbounds;
            relaxed_lower = sum(pos(f_lower(:) - data(:)));
            relaxed_upper = sum(pos(data(:) - f_upper(:)));
            relaxed_lower + relaxed_upper <= 50000;
            
        case 'Relaxed Bounds L2sq'   % ||(fl-u)+||_2^2 + ||(u-fu)+||_2^2
            f_lower = params.lowerbounds;
            f_upper = params.upperbounds;
            relaxed_lower = sum_square_pos(f_lower(:) - data(:));
            relaxed_upper = sum_square_pos(data(:) - f_upper(:));
            dataterm = (relaxed_lower) + (relaxed_upper);
    end

    
    % Define regularization term
    if alpha == 0
        reg = 0;
    else
        switch regularizer
            case 'L1'           % ||u||_1
                reg = norm( u(:) , 1 );

            case 'L2'           % ||u||_2
                reg = norm( u(:) , 2 );
                
            case 'L2_grad'      % ||Du||_2^2
                D = diffopn(sg,1,'forward','neumann');
                Du = D * u(:);
                reg = sum_square( Du );

            case 'TV'           % ||Du||_1
                D = diffopn(sg,1,'forward','neumann');
                Du = reshape( D * u(:) , prod(sg), 2 );
                reg = sum( norms( Du , 2 , 2 ) );
                
            case 'TV+L2'        % ||Du||_1 + 1e-4* ||u-u0||_2
                u0 = mean(img(:));
                D = diffopn(sg,1,'forward','neumann');
                Du = reshape( D * u(:) , prod(sg), 2 );
                reg = sum( norms( Du , 2 , 2 ) ) + 1e-4* norm( u(:) - u0, 2 );

            case 'TVLp-hom'     % ||Du-w||_1 + b/a* ||w||_p
                variable w(prod(sg), 2)
                p = params.p;
                b = params.beta;
                D = diffopn(sg,1,'forward','neumann');
                Du = reshape( D * u(:) , prod(sg), 2 );
                z = norms( w , 2 , 2);
                reg = sum( norms( Du-w , 2 , 2 ) )...
                    + b/alpha * sum( pow_pos( z , p ) );
        end
    end


    % Define objective function
    minimize( dataterm + alpha * reg );

cvx_end

% Fill stats struct for feedback
stats.cvx_status = cvx_status;
if ~strcmp(cvx_status, 'Solved')
    warning('\tProblem not solved.\tCVX Status: %s', cvx_status);
end
if contains(fidelity, 'Bounds')
    stats.f_lower = params.lowerbounds;
    stats.f_upper = params.upperbounds;
end
if contains(fidelity, 'Relaxed')
    stats.relaxedlower = nnz( f_lower(:)-data(:) > 1e-3 );
    stats.relaxedupper = nnz( data(:)-f_upper(:) > 1e-3 );
    if params.debugoutput
        disp(['Relaxed lower bounds: ', num2str(nnz((f_lower(:)-data(:))>1e-3)),...
              ' Relaxed upper bounds: ', num2str(nnz((data(:)-f_upper(:))>1e-3))]);
    end
end
stats.alpha = alpha;
stats.dataterm = dataterm;
stats.regularizer = reg;

% Return minimizer or value of dataterm (for discrepancy p.)
if argmin
    x = full(data);
else
    x = dataterm;
end

end

