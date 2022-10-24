close all
clear all
clc
set(groot,'DefaultAxesTickLabelInterpreter','latex');
% Scipt to generate figures of the beam model

ranges = [0:0.1:51];
mu = 30;
dist = hit(ranges, mu, 1) + short(ranges, 0.06, mu) + miss(ranges, 2, 50) + 0.025*ones(size(ranges));
dist(ranges >= 50) = 0;


figure
fh = plot(ranges,dist,'b', "LineWidth",1.5);
fixplot(fh)

figure
subplot(2,2,1)
fh = plot(ranges,hit(ranges, mu, 1),'b', "LineWidth",1.5);
xlabel("a) Gaussian distribution $$p_{hit}$$", Interpreter="latex")
fixplot(fh)

subplot(2,2,2)
fh = plot(ranges,short(ranges, 0.06, mu),'b', "LineWidth",1.5);
xlabel("b) Exponential distribution $$p_{short}$$", Interpreter="latex")
fixplot(fh)
ylim([0,0.2])

subplot(2,2,3)
fh = plot(ranges,0.025*ones(size(ranges)),'b', "LineWidth",1.5);
xlabel("c) Uniform distribution $$p_{rand}$$", Interpreter="latex")
fixplot(fh)
ylim([0,0.2])

subplot(2,2,4)
fh = plot(ranges,miss(ranges, 2, 50),'b', "LineWidth",1.5);
xlabel("d) Uniform distribution $$p_{max}$$", Interpreter="latex")
fixplot(fh)


function fixplot(fh)
    ylim([0,0.5])
    xlim([0,51])
    axis = get(fh,'Parent');
    yTick = get(axis,'YTick');
    set(axis,'YTick',[]);
    set(axis,'XTick',[30, 50]);
    set(axis, 'XTickLabel', {"$$z^{k*}_t$$","$$z_{max}$$"});
    set(axis, 'FontSize', 24);
    % xlabel("p(z^k_t | x_t, m)", 'FontSize',14)
    ylabel('$$p(z^k_t | x_t, m)$$','Interpreter','latex', 'FontSize',24);
end



function data = hit(vec, mu, var)
    data = (1/sqrt(2*pi*var))*exp(-0.5*((vec - mu).^2)/var^2);
end


function data = short(vec, lambda, hitrange)
    data = lambda*exp(-lambda.*vec);
    idx = vec > hitrange;
    data(idx) = 0;
end

function data = miss(vec, lambda, maxrange)
    data = zeros(size(vec));
    idx = abs(vec - maxrange) < lambda;
    data(idx) = 0.2;
end

