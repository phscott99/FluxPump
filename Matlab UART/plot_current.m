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

writeline(STM32_COM, "?I offset");
offset = double(read(STM32_COM, 1, 'int16'));

writeline(STM32_COM, "?I conv");
conversion = double(read(STM32_COM, 1, 'single'));

writeline(STM32_COM, "?T div");
divider = double(read(STM32_COM, 1, 'uint16'));

writeline(STM32_COM, "?T spc");
samplesPerCycle = double(read(STM32_COM, 1, 'uint32'));

writeline(STM32_COM, "#I read");
current = double(read(STM32_COM, samplesPerCycle/divider, 'int16'));

current = current*conversion;
time = linspace(0, samplesPerCycle/1000000, samplesPerCycle/divider)';

clf
hold on
plot(time,current);
hold off

