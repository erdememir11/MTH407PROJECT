%% MTH407 Donem Projesi
% Akilli Fabrikalarda Otonom Robot (AGV) Takibi
% TDOA olcum modeli + LSE ilklendirme + EKF takip algoritmasi

clear; clc; close all;

rng(407);

cfg = defaultConfig();
ensureOutputFolder(cfg.outputFolder);

truth = simulateAgvTrajectory(cfg);
measurements = generateTdoaMeasurements(truth, cfg, cfg.sensorsGood);

init = initializeWithLse(measurements, cfg, cfg.sensorsGood);
ekf = runEkfTracker(measurements, init, cfg, cfg.sensorsGood);

metricsGood = computeMetrics(truth, ekf);
geometryResults = analyzeSensorGeometry(cfg, truth);

saveOutputs(cfg, truth, measurements, ekf, metricsGood, geometryResults);
plotFactoryTracking(cfg, truth, measurements, ekf);
plotTimeSeries(cfg, truth, ekf);
plotGeometryAnalysis(cfg, geometryResults);

fprintf('\nAGV TDOA-EKF simülasyonu tamamlandı.\n');
fprintf('Ana senaryo konum RMSE: %.3f m\n', metricsGood.rmsePosition);
fprintf('Ana senaryo ortalama NLOS anchor sayisi: %.2f / %d\n', ...
    mean(sum(measurements.isNlos, 2)), size(cfg.sensorsGood, 1));
fprintf('Ciktilar: %s\n\n', cfg.outputFolder);

%% Local functions

function cfg = defaultConfig()
    cfg.outputFolder = "outputs";

    cfg.c = 299792458;              % UWB sinyal yayilma hizi [m/s]
    cfg.dt = 0.2;                   % Olcum araligi [s]
    cfg.N = 260;                    % Zaman adimi sayisi
    cfg.factorySize = [42, 26];     % [genislik, yukseklik] [m]

    cfg.sigmaToaLos = 0.20 / cfg.c;     % Acik alan UWB TOA std: 20 cm
    cfg.sigmaToaNlos = 1.20 / cfg.c;    % Engel etkili TOA std: 120 cm
    cfg.sigmaAccel = 0.45;              % Surec ivme std [m/s^2]

    % UWB anchor antenleri: duvar/kolon uzerinde bilinen sabit noktalar.
    cfg.sensorsGood = [
         1.5,  1.5;
        40.0,  1.2;
        41.0, 24.5;
         1.2, 24.0;
        20.5,  3.2;
        31.5, 17.5
    ];

    % Kotu geometri: anchorlar neredeyse ayni duvar hattinda.
    cfg.sensorsPoor = [
         2.0,  1.0;
         9.0,  1.2;
        16.0,  0.9;
        23.0,  1.1;
        30.0,  0.8;
        38.0,  1.0
    ];

    % [x, y, w, h] dikdortgen fabrika engelleri/hat makineleri.
    cfg.obstacles = [
         8.0,  6.0,  5.5,  4.0;
        18.0, 10.0,  7.0,  3.5;
        29.0,  5.0,  4.0,  8.0;
        12.0, 18.0, 10.0,  3.0
    ];

    cfg.referenceSensor = 1;
    cfg.lseIterations = 20;
    cfg.geometryMonteCarloRuns = 30;
end

function ensureOutputFolder(folderName)
    if ~exist(folderName, 'dir')
        mkdir(folderName);
    end
end

function truth = simulateAgvTrajectory(cfg)
    t = (0:cfg.N-1)' * cfg.dt;
    x = zeros(cfg.N, 4);
    x(1, :) = [4.0, 4.0, 1.05, 0.55];

    for k = 2:cfg.N
        tk = t(k);
        ax = 0.22 * sin(0.23 * tk) - 0.08 * sin(0.71 * tk);
        ay = 0.18 * cos(0.19 * tk) + 0.06 * sin(0.53 * tk);

        x(k, 3:4) = x(k-1, 3:4) + cfg.dt * [ax, ay];
        speed = norm(x(k, 3:4));
        if speed > 1.8
            x(k, 3:4) = 1.8 * x(k, 3:4) / speed;
        end

        x(k, 1:2) = x(k-1, 1:2) + cfg.dt * x(k, 3:4);

        margin = 1.0;
        if x(k, 1) < margin || x(k, 1) > cfg.factorySize(1) - margin
            x(k, 3) = -0.85 * x(k, 3);
            x(k, 1) = min(max(x(k, 1), margin), cfg.factorySize(1) - margin);
        end
        if x(k, 2) < margin || x(k, 2) > cfg.factorySize(2) - margin
            x(k, 4) = -0.85 * x(k, 4);
            x(k, 2) = min(max(x(k, 2), margin), cfg.factorySize(2) - margin);
        end
    end

    truth.t = t;
    truth.state = x;
    truth.position = x(:, 1:2);
end

function measurements = generateTdoaMeasurements(truth, cfg, sensors)
    M = size(sensors, 1);
    ref = cfg.referenceSensor;
    other = setdiff(1:M, ref);
    K = size(truth.position, 1);

    zRangeDiff = zeros(K, M-1);
    zTimeDiff = zeros(K, M-1);
    R = zeros(M-1, M-1, K);
    isNlos = false(K, M);

    for k = 1:K
        p = truth.position(k, :);
        d = vecnorm(sensors - p, 2, 2);

        sigmaToa = cfg.sigmaToaLos * ones(M, 1);
        for m = 1:M
            if lineIntersectsAnyObstacle(p, sensors(m, :), cfg.obstacles)
                sigmaToa(m) = cfg.sigmaToaNlos;
                isNlos(k, m) = true;
            end
        end

        noisyToa = d / cfg.c + sigmaToa .* randn(M, 1);
        tdoa = noisyToa(other) - noisyToa(ref);

        zTimeDiff(k, :) = tdoa';
        zRangeDiff(k, :) = (cfg.c * tdoa)';

        rangeVar = cfg.c^2 * (sigmaToa(other).^2 + sigmaToa(ref).^2);
        R(:, :, k) = diag(rangeVar);
    end

    measurements.zRangeDiff = zRangeDiff;
    measurements.zTimeDiff = zTimeDiff;
    measurements.R = R;
    measurements.isNlos = isNlos;
end

function init = initializeWithLse(measurements, cfg, sensors)
    count = min(8, size(measurements.zRangeDiff, 1));
    pHat = zeros(count, 2);
    pCov = zeros(2, 2, count);

    for k = 1:count
        [pHat(k, :), pCov(:, :, k)] = estimatePositionLse( ...
            measurements.zRangeDiff(k, :)', measurements.R(:, :, k), cfg, sensors);
    end

    time = (0:count-1)' * cfg.dt;
    vx = polyfit(time, pHat(:, 1), 1);
    vy = polyfit(time, pHat(:, 2), 1);

    init.x0 = [pHat(1, 1); pHat(1, 2); vx(1); vy(1)];
    init.P0 = zeros(4);
    init.P0(1:2, 1:2) = pCov(:, :, 1) + 0.25 * eye(2);
    init.P0(3:4, 3:4) = diag([0.8, 0.8].^2);
    init.lsePositions = pHat;
end

function [p, covP] = estimatePositionLse(z, R, cfg, sensors)
    p = mean(sensors, 1)';
    W = inv(R);

    for iter = 1:cfg.lseIterations
        [h, Hpos] = tdoaMeasurementModel(p, sensors, cfg.referenceSensor);
        residual = z - h;
        step = (Hpos' * W * Hpos + 1e-6 * eye(2)) \ (Hpos' * W * residual);
        p = p + step;
        p(1) = min(max(p(1), 0.2), cfg.factorySize(1) - 0.2);
        p(2) = min(max(p(2), 0.2), cfg.factorySize(2) - 0.2);
        if norm(step) < 1e-5
            break;
        end
    end

    [~, Hpos] = tdoaMeasurementModel(p, sensors, cfg.referenceSensor);
    covP = inv(Hpos' * W * Hpos + 1e-6 * eye(2));
end

function ekf = runEkfTracker(measurements, init, cfg, sensors)
    K = size(measurements.zRangeDiff, 1);
    x = zeros(K, 4);
    P = zeros(4, 4, K);
    innovationNorm = zeros(K, 1);

    x(1, :) = init.x0';
    P(:, :, 1) = init.P0;

    dt = cfg.dt;
    F = [
        1, 0, dt, 0;
        0, 1, 0, dt;
        0, 0, 1, 0;
        0, 0, 0, 1
    ];
    G = [
        0.5 * dt^2, 0;
        0, 0.5 * dt^2;
        dt, 0;
        0, dt
    ];
    Q = (cfg.sigmaAccel^2) * (G * G');

    I = eye(4);
    for k = 2:K
        xPred = F * x(k-1, :)';
        PPred = F * P(:, :, k-1) * F' + Q;

        [h, Hpos] = tdoaMeasurementModel(xPred(1:2), sensors, cfg.referenceSensor);
        H = [Hpos, zeros(size(Hpos, 1), 2)];
        Rk = measurements.R(:, :, k);
        y = measurements.zRangeDiff(k, :)' - h;
        S = H * PPred * H' + Rk;
        Kk = PPred * H' / S;

        xUpd = xPred + Kk * y;
        PUpd = (I - Kk * H) * PPred * (I - Kk * H)' + Kk * Rk * Kk';

        xUpd(1) = min(max(xUpd(1), 0.0), cfg.factorySize(1));
        xUpd(2) = min(max(xUpd(2), 0.0), cfg.factorySize(2));

        x(k, :) = xUpd';
        P(:, :, k) = PUpd;
        innovationNorm(k) = sqrt(y' / S * y);
    end

    ekf.state = x;
    ekf.position = x(:, 1:2);
    ekf.P = P;
    ekf.innovationNorm = innovationNorm;
end

function [h, Hpos] = tdoaMeasurementModel(p, sensors, ref)
    p = p(:)';
    M = size(sensors, 1);
    other = setdiff(1:M, ref);

    diff = p - sensors;
    d = vecnorm(diff, 2, 2);
    d = max(d, 1e-6);

    h = d(other) - d(ref);
    Hpos = zeros(M-1, 2);
    for row = 1:numel(other)
        i = other(row);
        Hpos(row, :) = diff(i, :) / d(i) - diff(ref, :) / d(ref);
    end
end

function hit = lineIntersectsAnyObstacle(p1, p2, obstacles)
    hit = false;
    for i = 1:size(obstacles, 1)
        if segmentIntersectsRect(p1, p2, obstacles(i, :))
            hit = true;
            return;
        end
    end
end

function hit = segmentIntersectsRect(p1, p2, rect)
    x = rect(1); y = rect(2); w = rect(3); h = rect(4);
    xmin = x; xmax = x + w; ymin = y; ymax = y + h;

    if pointInRect(p1, rect) || pointInRect(p2, rect)
        hit = true;
        return;
    end

    corners = [xmin ymin; xmax ymin; xmax ymax; xmin ymax];
    edges = [1 2; 2 3; 3 4; 4 1];
    hit = false;
    for e = 1:4
        q1 = corners(edges(e, 1), :);
        q2 = corners(edges(e, 2), :);
        if segmentsIntersect(p1, p2, q1, q2)
            hit = true;
            return;
        end
    end
end

function inside = pointInRect(p, rect)
    inside = p(1) >= rect(1) && p(1) <= rect(1) + rect(3) && ...
             p(2) >= rect(2) && p(2) <= rect(2) + rect(4);
end

function hit = segmentsIntersect(a, b, c, d)
    hit = ccw(a, c, d) ~= ccw(b, c, d) && ccw(a, b, c) ~= ccw(a, b, d);
end

function value = ccw(a, b, c)
    value = (c(2) - a(2)) * (b(1) - a(1)) > (b(2) - a(2)) * (c(1) - a(1));
end

function metrics = computeMetrics(truth, ekf)
    err = ekf.position - truth.position;
    posErr = vecnorm(err, 2, 2);
    metrics.rmsePosition = sqrt(mean(posErr.^2));
    metrics.meanPositionError = mean(posErr);
    metrics.maxPositionError = max(posErr);
end

function geometryResults = analyzeSensorGeometry(cfg, truth)
    names = ["Iyi geometri"; "Zayif geometri"];
    sensorSets = {cfg.sensorsGood, cfg.sensorsPoor};
    rmse = zeros(2, cfg.geometryMonteCarloRuns);
    meanCond = zeros(2, 1);

    for s = 1:2
        sensors = sensorSets{s};
        condSeries = zeros(size(truth.position, 1), 1);
        for k = 1:size(truth.position, 1)
            [~, Hpos] = tdoaMeasurementModel(truth.position(k, :), sensors, cfg.referenceSensor);
            condSeries(k) = cond(Hpos' * Hpos + 1e-9 * eye(2));
        end
        meanCond(s) = mean(condSeries);

        for mc = 1:cfg.geometryMonteCarloRuns
            meas = generateTdoaMeasurements(truth, cfg, sensors);
            init = initializeWithLse(meas, cfg, sensors);
            ekf = runEkfTracker(meas, init, cfg, sensors);
            m = computeMetrics(truth, ekf);
            rmse(s, mc) = m.rmsePosition;
        end
    end

    geometryResults.names = names;
    geometryResults.meanRmse = mean(rmse, 2);
    geometryResults.stdRmse = std(rmse, 0, 2);
    geometryResults.meanCondition = meanCond;
end

function saveOutputs(cfg, truth, measurements, ekf, metrics, geometryResults)
    resultTable = table( ...
        truth.t, ...
        truth.position(:, 1), truth.position(:, 2), ...
        ekf.position(:, 1), ekf.position(:, 2), ...
        vecnorm(ekf.position - truth.position, 2, 2), ...
        sum(measurements.isNlos, 2), ...
        'VariableNames', {'time_s', 'true_x_m', 'true_y_m', ...
        'estimated_x_m', 'estimated_y_m', 'position_error_m', 'nlos_anchor_count'});
    writetable(resultTable, fullfile(cfg.outputFolder, 'tracking_results.csv'));

    summaryTable = table( ...
        metrics.rmsePosition, metrics.meanPositionError, metrics.maxPositionError, ...
        'VariableNames', {'rmse_position_m', 'mean_position_error_m', 'max_position_error_m'});
    writetable(summaryTable, fullfile(cfg.outputFolder, 'main_metrics.csv'));

    geometryTable = table( ...
        geometryResults.names, geometryResults.meanRmse, geometryResults.stdRmse, ...
        geometryResults.meanCondition, ...
        'VariableNames', {'geometry', 'mean_rmse_m', 'std_rmse_m', 'mean_condition_number'});
    writetable(geometryTable, fullfile(cfg.outputFolder, 'geometry_analysis.csv'));
end

function plotFactoryTracking(cfg, truth, measurements, ekf)
    fig = figure('Color', 'w', 'Name', 'Factory tracking map');
    hold on; grid on; axis equal;
    xlim([0 cfg.factorySize(1)]); ylim([0 cfg.factorySize(2)]);

    for i = 1:size(cfg.obstacles, 1)
        rectangle('Position', cfg.obstacles(i, :), 'FaceColor', [0.72 0.72 0.72], ...
            'EdgeColor', [0.25 0.25 0.25], 'LineWidth', 1.2);
    end

    scatter(cfg.sensorsGood(:, 1), cfg.sensorsGood(:, 2), 70, 'filled', ...
        'MarkerFaceColor', [0.05 0.30 0.75], 'DisplayName', 'UWB anchor');
    text(cfg.sensorsGood(:, 1) + 0.35, cfg.sensorsGood(:, 2) + 0.35, ...
        compose('A%d', 1:size(cfg.sensorsGood, 1)), 'Color', [0.05 0.20 0.55]);

    nlosCount = sum(measurements.isNlos, 2);
    scatter(truth.position(nlosCount > 0, 1), truth.position(nlosCount > 0, 2), ...
        12, [0.90 0.45 0.05], 'filled', 'DisplayName', 'NLOS olcum ani');
    plot(truth.position(:, 1), truth.position(:, 2), 'k-', 'LineWidth', 2.0, ...
        'DisplayName', 'Gercek AGV yolu');
    plot(ekf.position(:, 1), ekf.position(:, 2), '-', 'Color', [0.80 0.05 0.18], ...
        'LineWidth', 1.7, 'DisplayName', 'EKF tahmini');

    xlabel('x [m]'); ylabel('y [m]');
    title('2B fabrika zemini: TDOA tabanli AGV takibi');
    legend('Location', 'bestoutside');
    exportgraphics(fig, fullfile(cfg.outputFolder, 'factory_tracking_map.png'), 'Resolution', 160);
end

function plotTimeSeries(cfg, truth, ekf)
    err = vecnorm(ekf.position - truth.position, 2, 2);

    fig = figure('Color', 'w', 'Name', 'Tracking time series');
    tiledlayout(3, 1, 'TileSpacing', 'compact');

    nexttile;
    plot(truth.t, truth.position(:, 1), 'k-', 'LineWidth', 1.6); hold on; grid on;
    plot(truth.t, ekf.position(:, 1), 'r--', 'LineWidth', 1.4);
    ylabel('x [m]');
    legend('Gercek', 'Tahmin', 'Location', 'best');

    nexttile;
    plot(truth.t, truth.position(:, 2), 'k-', 'LineWidth', 1.6); hold on; grid on;
    plot(truth.t, ekf.position(:, 2), 'r--', 'LineWidth', 1.4);
    ylabel('y [m]');

    nexttile;
    plot(truth.t, err, 'Color', [0.10 0.45 0.35], 'LineWidth', 1.4); grid on;
    xlabel('zaman [s]'); ylabel('konum hatasi [m]');
    title(sprintf('RMSE = %.2f m', sqrt(mean(err.^2))));

    exportgraphics(fig, fullfile(cfg.outputFolder, 'tracking_time_series.png'), 'Resolution', 160);
end

function plotGeometryAnalysis(cfg, geometryResults)
    fig = figure('Color', 'w', 'Name', 'Sensor geometry analysis');
    tiledlayout(1, 2, 'TileSpacing', 'compact');

    nexttile;
    bar(geometryResults.meanRmse, 'FaceColor', [0.15 0.45 0.75]); grid on;
    hold on;
    errorbar(1:2, geometryResults.meanRmse, geometryResults.stdRmse, ...
        'k.', 'LineWidth', 1.2);
    set(gca, 'XTickLabel', geometryResults.names);
    ylabel('Ortalama RMSE [m]');
    title('Sensor geometrisi etkisi');

    nexttile;
    semilogy(geometryResults.meanCondition, 'o-', 'LineWidth', 1.6, ...
        'Color', [0.70 0.20 0.20]); grid on;
    set(gca, 'XTick', 1:2, 'XTickLabel', geometryResults.names);
    ylabel('Ortalama cond(H^T H)');
    title('TDOA geometri kosullanmasi');

    exportgraphics(fig, fullfile(cfg.outputFolder, 'sensor_geometry_analysis.png'), 'Resolution', 160);
end
