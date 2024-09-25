function initializeUASTraj(obj)
  num_uas = obj.uas_config.num_uas;
  t0 = obj.sim_config.t0;
  tf = obj.sim_config.tf;
  % Get the extent of this lane system
  %             [minx, miny, maxx, maxy] = obj.lbsd.getEnvelope();
  failed_i = zeros(num_uas,1);
  num_fail = 0;
  num_success = 0;
  success_i = zeros(num_uas,1);
  res_times = zeros(num_uas,1);
  res_delays = zeros(num_uas,1);
  res_mission_times = zeros(num_uas,1);
  for i = 1:num_uas
    uas_i = obj.uas_list(i);
    if isempty(obj.sim_config.single_lane)
      max_iter = 1000;
      % Create random start and end points
      ok = false;
      t_i = 0;
      while ~ok && t_i < max_iter
        %                         x0 = [minx+(maxx-minx)*rand(),miny+(maxy-miny)*rand()];
        %                         xf = [minx+(maxx-minx)*rand(),miny+(maxy-miny)*rand()];
        % Create a trajectory based on this UAS capabilities
%         [traj, lane_ids, vert_ids, toa_s, ok] = ...
%           uas_i.createTrajectoryRandVerts(obj.sim_config.fit_traj);
        [traj, lane_ids, vert_ids, toa_s, ok] = ...
          uas_i.createTrajectoryRandVertsNoRates(obj.sim_config.fit_traj);
        t_i = t_i + 1;
      end
      if ~ok
        error("Failed to create proper trajectory");
      end
    else
      l = obj.sim_config.single_lane;
      [traj, lane_ids, vert_ids, toa_s] = ...
        uas_i.createTrajectoryLane(l, obj.sim_config.fit_traj);
    end
    % Generate a random request time
    r = t0+(tf-t0)*rand();
    uas_i.r_s = r;
    % Move the time-of-arrival of the trajectory to this time
    toa_s = toa_s+r;
    % The earliest and latest possible release
    % For renyi set to the request time
    r_t0 = max(r-uas_i.flex, t0);
    r_tf = min(r+uas_i.flex, tf);

    % Reserve the trajectory
    timerVal = tic;
    [ok, res_ids, res_toa_s] = ...
      obj.lbsd.reserveLBSDTrajectory(lane_ids, uas_i.id, toa_s, ...
      uas_i.h_d, r_t0, r_tf);
    res_times(i) = toc(timerVal);
    if ok
      % The trajectory was scheduled successfully
      res_mission_times(i) = res_toa_s(end)-res_toa_s(1);
      res_delays(i) = abs(res_toa_s(1)-r);
      uas_i.res_ids = res_ids;
      uas_i.traj = traj;
      uas_i.toa_s = res_toa_s;
      step_cnt = ceil(...
        (res_toa_s(end)-res_toa_s(1))*obj.step_rate_hz...
        )+1;
      uas_i.exec_traj = zeros(step_cnt,3);
      uas_i.exec_orient = quaternion(zeros(step_cnt,4));
      uas_i.exec_vel = zeros(step_cnt,3);
      % Executed trajectory acceleration (nx3)
      uas_i.exec_accel = zeros(step_cnt,3);
      % Executed trajectory angular velocity (nx3)
      uas_i.exec_ang_vel = zeros(step_cnt,3);

      uas_i.set_point_hz = obj.step_rate_hz;
      num_success = num_success + 1;
      success_i(num_success) = i;
    else
      num_fail = num_fail + 1;
      failed_i(num_fail) = i;
      uas_i.failed_to_schedule = true;
    end
  end
  if num_fail < num_uas
    failed_i(num_fail+1:end) = [];
  end
  if num_success < num_uas
    success_i(num_success+1:end) = [];
  end

  mission_times = res_mission_times(success_i);
  m_time_mean = mean(mission_times);
  m_time_var = var(mission_times);
  m_time_median = median(mission_times);
  m_time_max = max(mission_times);
  m_time_min = min(mission_times);

  res_mean = mean(res_times);
  res_var = var(res_times);
  res_median = median(res_times);
  res_max = max(res_times);
  res_min = min(res_times);

  delays = res_delays(success_i);
  delay_mean = mean(delays);
  delay_var = var(delays);
  delay_median = median(delays);
  delay_max = max(delays);
  delay_min = min(delays);

  obj.sim_metrics.mission_time.max = m_time_max;
  obj.sim_metrics.mission_time.min = m_time_min;
  obj.sim_metrics.mission_time.mean = m_time_mean;
  obj.sim_metrics.mission_time.median = m_time_median;
  obj.sim_metrics.mission_time.var = m_time_var;

  obj.sim_metrics.delay_time.max = delay_max;
  obj.sim_metrics.delay_time.min = delay_min;
  obj.sim_metrics.delay_time.mean = delay_mean;
  obj.sim_metrics.delay_time.median = delay_median;
  obj.sim_metrics.delay_time.var = delay_var;

  obj.sim_metrics.reservation_time.max = res_max;
  obj.sim_metrics.reservation_time.min = res_min;
  obj.sim_metrics.reservation_time.mean = res_mean;
  obj.sim_metrics.reservation_time.median = res_median;
  obj.sim_metrics.reservation_time.var = res_var;
  %             obj.sim_metrics.failed_flights_ids = failed_i;
  obj.sim_metrics.num_failed_flights = num_fail;
  %             obj.sim_metrics.success_flights_ids = success_i;
  obj.sim_metrics.num_success_flights = num_success;

end
