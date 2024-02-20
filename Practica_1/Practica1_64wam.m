% 64-QAM (Quadrature Amplitude Modulation)

% En el caso de la 64-QAM, los números complejos son -7 -7i, -7 -5i,...,-7 +7i,
% -5 -7i,...,-5 +7i,...,+7 +7i
TC = [];
for i = -7:2:7
    for j = -7:2:7
        TC = [TC i+j*1j];
    end
end


% Generar un vector de bits aleatorios de longitud N
N = 4800;
bits = randi([0, 1], 1, N);
disp(['BITS : ' mat2str(bits)]);

M = length(TC); %simbolos de modulacion
k = log2(M);   % Número de bits por símbolo


bits_reshape = reshape(bits', k,[]);

ind = bit2int(bits_reshape,k); % convierte cada columna de X en un entero
simbM=TC(ind+1);

%ENERGIA MEDIA DE SIMBOLO
Es = sum(abs(TC).^2)/M;

%ENERGIA MEDIA DE BIT
Eb = Es/k;

% Definir el valor de Eb/N0 en decibelios
x = 0; % dB

% Calcular el valor de Eb/N0 correspondiente
EbN0 = 10^(x/10);

% Calcular N0
N0 = Eb/EbN0;

%GENERAR VECTOR DE RUIDO COMPLEJO AWGN (Additive White Gaussian Noise)
n = (randn(size(simbM)) + 1j*randn(size(simbM)))*sqrt(N0/2);


%VECTOR DE SIMBOLOS DESPUES DE HABER SIDO RETRANSMITIDO POR EL CANAL AWGN 
simbR=simbM+n;


%DISTANCIA EUCLIDEA ENTRE LOS VALORES RECIBIDOS Y LOS SIMBOLOS EN LA CONSTELACION QPSK 
dis_euclide=abs(repmat(simbR,M,1)-repmat(TC.',1,length(simbR)));
[val_min,pos_min]=min(dis_euclide,[],1);
simbR_demodulado=TC(pos_min);

%CONVERTIR SIMBOLOS DEMODULADOS A BITS
bits_demodulados=int2bit(pos_min-1,k);
bits_demodulados=reshape(bits_demodulados,[],N);

% Comparar los bits demodulados con los originales
errores = sum(bits ~= bits_demodulados);

% Calcular la tasa de error de bits (BER)
BER = errores/N;

%APARTADO GRAFICA DE VALORES BER

% Definir un rango de valores para Eb/N0 en dB
EbN0dB = [-100 -80 -70 -60 -50 -40 -30 -20 -10 0];

% Inicializar un vector para almacenar los valores de BER
BER = zeros(size(EbN0dB));


% Calcular la BER para cada valor de Eb/N0
for i = 1:length(EbN0dB)
    % Calcular el valor de Eb/N0 correspondiente
    EbN0 = 10^(EbN0dB(i)/10);

    % Calcular N0
    N0 = Eb/EbN0;

    %GENERAR VECTOR DE RUIDO COMPLEJO AWGN (Additive White Gaussian Noise)
    n = (randn(size(simbM)) + 1j*randn(size(simbM)))*sqrt(N0/2);

    %VECTOR DE SIMBOLOS DESPUES DE HABER SIDO RETRANSMITIDO POR EL CANAL
    simbR = simbM + n;

     % Calcular las distancias euclídeas entre los valores recibidos y los símbolos de la constelación 16-QAM
    dis_euclide = abs(repmat(simbR,M,1) - repmat(TC.',size(simbR)));


    % Encontrar el símbolo de la constelación más cercano para cada valor recibido
    [~, pos_min] = min(dis_euclide,[],1);

    simbR_demodulado=TC(pos_min);

    %CONVERTIR SIMBOLOS DEMODULADOS A BITS
    bits_demodulados=int2bit(pos_min-ones(size(pos_min)),k);
    bits_demodulados=reshape(bits_demodulados,[],N);

    % Comparar los bits demodulados con los originales
    errores = sum(bits ~= bits_demodulados);

    % Calcular la tasa de error de bits (BER)
    BER(i) = errores/N;
    
end  