if exist('STM32_COM', 'var') == 0
    STM32_COM = serialport("COM4",3953488,'parity','even');
    configureTerminator(STM32_COM, "CR/LF", "CR");
    writeline(STM32_COM, "#echo");
    if ~strcmp(readline(STM32_COM), sprintf("#echo echo"))
        clearvars STM32_COM
        error("unable to successfully communicate with STM32"); 
    end
end

flush(STM32_COM);

writeline(STM32_COM, "#T a 0");
pause(1);

N_steps = 1000;
for x = 1:N_steps
    str = sprintf("#T a %f", x*15/N_steps);
    writeline(STM32_COM, str);
    
    pause(0.1);
    
    writeline(STM32_COM, "?I pk");
    I_peak(x) = double(read(STM32_COM, 1, 'single'));
    writeline(STM32_COM, "?I rms");
    I_rms(x) = double(read(STM32_COM, 1, 'single'));
    writeline(STM32_COM, "?P duty");
    P_duty(x) = double(read(STM32_COM, 1, 'uint32'))/10000;

    clf
    hold on
    plot(I_peak);
    plot(I_rms);
    plot(P_duty);
    hold off
end

