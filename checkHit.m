function [isOnSurface2d , gamma_] = checkHit( P1 , P2 , P3, unitVec, startOrigin )
    % from lidar_origin to u, extrapolate a point to hit W and L
    % startOrigin + gamma_ * unitVec shall be a point on the line P1P2
    % gamma_ shall be a nonnegative value
    % if the gamma_ value is too large,
    % we return -1 as parallel / nearly parallel wall
    % are not considered.  isOnSurface2d is also false in this case

    isOnSurface2d = false;

    xo = startOrigin(1); yo = startOrigin(2); zo = startOrigin(3);
    xu = unitVec(1); yu = unitVec(2); zu = unitVec(3);

    %    a x  +  b y  +  c z  +  d  =  0
    d = 1;
    P = [P1; P2; P3]';
    abc = -P'\ones(3,1);
    a = abc(1); b = abc(2); c = abc(3);
    % we use ones(3,1) as the system response vector
    % thus we enforce d = 1
    % Assumption taken: origin (0,0,0) is not on the plane

    axo_plus_byo_plus_czo = a * xo + b * yo + c * zo;
    axu_plus_byu_plus_czu = a * xu + b * yu + c * zu;
    if (abs(axu_plus_byu_plus_czu) < 1E-3)
        gamma_ = -1;
        return;
    else
        gamma_ = - ( axo_plus_byo_plus_czo + d ) / axu_plus_byu_plus_czu;
        if (gamma_ > 1E3)
            gamma_ = -1;
            return;
        end
    end
    
    if (gamma_ < 0)
        gamma_ = -1;
        return;
    end

    q = [ xo ; yo ; zo ] + gamma_ * [ xu ; yu ; zu];
    %figure; scatter3([P1(1),P2(1),P3(1)], [P1(2),P2(2),P3(2)], [P1(3),P2(3),P3(3)], 'bo'); hold on; scatter3(q(1), q(2), q(3), 'rx');
    % -------------------------------------------- %
    % Let f(s) = 0.5 * || P*s - q ||_2^2           %
    % solve the problem :                          %
    % \underset{min}{s}    f(s)                    %
    %     such that                                %
    %                     s >= 0                   %
    %                   sum(s) = 1                 %
    % -------------------------------------------- %
    % in order to check if point q resides in the simplex formed by P
    % if q is in P, then objective f(s) should be 0
    % otherwise, f(s) > 0
    
    s = P\q;
    if (~sum(s < 0) && (abs(sum(s) - 1) < 1E-3) && (norm(P*s-q,2) < 1E-3))
        isOnSurface2d = true;
    end
        
%     s   = ones(3,1)/3;
%     PtP = P'*P;
%     L   = max(eig(PtP)); % Lipschitz constant
%     obj_curr = +Inf;
%     for itr = 1 : 1E5
%         GRAD = PtP*s - P'*q;
%         s = SimplexProj((s - GRAD / L)')';
%         Ps_minus_q = P*s - q;
%         obj_prev = obj_curr;
%         obj_curr = Ps_minus_q'*Ps_minus_q;
%         if (obj_curr < 1E-3)
%             break;
%         end
%         if (abs(obj_curr - obj_prev) < 1E-6)
%             break;
%         end
%     end ; clear itr
% 
%     if (obj_curr < 1E-3)
%         isOnSurface2d = true;
%     end
end
