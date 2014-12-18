function [cur, pose, pose_dot, raw, dp] = manela_icp(map, raw, pose, pose_dot, dp, rejection_setting, converge_metric)
    ii = 0;
    temp_map  = [];
    temp_scan = [];
    
    while true
        ii = ii + 1;

        % Remove all points that do not have a 'near' match
        [temp_scan, rej_map] = killoutliers(map, raw, rejection_setting);
        [temp_map, rej_raw] = killoutliers(temp_scan, map, rejection_setting);

        %calculate union of rejections, may be useful if your
        %calcutating rejection for a scan-to-map, but not really for a
        %scan-to-scan
        %rej_union = rej_map * rej_raw;

        % Execute ICP
        [TR, TT] = call_icp1(temp_map, temp_scan);

        % Transform current scan and pose estimate to new location.
        TR = TR(1:2,1:2); % Cheap way to only select z rotation
        TT = repmat(TT(1:2), 1, size(raw,2));

        theta = acos(TR(1));

        raw = real(TR * raw + TT);
        dp = real(TR * dp + TT(:,1:2));

        %get the difference in TT so we know when to converge the icp
        %algorithm.  This is a pretty schotty way of convergence, as it
        %doesn't take into account if the value bounced around the setpoint
        %but it works for this application
        dTT = mag(TT(:,1));

        %we need to run the loop at least once before we check this
        if ii > 1
            %Check for convergence
            if abs(dTT) < converge_metric
                tt = dp(:,2) - dp(:,1);
                [theta, ~] = cart2pol(tt(1), tt(2));
                cur = ([dp(:,1);theta]);
                pose = [pose cur];
                pose_dot = pose(:,end) - pose(:,end-1);
                break;
            end
        end
    end

    if rej_raw > 0.6
        warning('you done messed up');
    end