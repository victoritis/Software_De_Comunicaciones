%Crear arrays
a = [ 1 3 4];

a = 10:-0.5:-1;



%Acceder arrays y matrices

%acceder segundo elemento del vector
%a(2);

%accerder ultimo elemento del vector
%a(end);

%

%Crea un array con los primeros 4 elementos de a
a (1;4);

%que empiece en la posicion 2 y vaya accediendo a las pares y las guarde en
%un array
%a(2;2;end);







%OPERADORES MATEMATICOS

a = randi(4,1,3);
b = randi(4,3,3);
c = randi(4,size(b));

d = b + c;
d = 2 + b;

%producto de matrices estandar DE TODA LA VIDA
d = a*b;

%Otro tipo de producto(ELEMENTO A ELEMENTO)
d = b.*c;

d = 2*b;
d = b/2;

%Se dividen cada elemento de B por su correspndiente en C
d = b./c;

d = b/c; %d b*inv(c)
d = b\c; %d inv(b)*c

%Se eleva cada elemento al 2
d = b.^2;

%Se eleva la matriz b a ella misma, EXPONENCIACION DE MATRICES
d=b^2;


%DEBEN SER LAS DIMENSIONES IGUALES
a + b;


b == 2;
b>2 & b <= 4;
b(b > 2 & b < 4)) = -3;


%CUANDO BORRAMOS ELEMENTOS A UNA MATRIZ SE CONVERTIRA EN UN VECTOR Y SE
%ESCRIBIRA POR ORDEN DE COLUMNA
a(a<2 | a>8) = [];


%Suma POR COLUMNAS
s = sum(b,1);
%EQUIVALENTE A=
s = sum(b);

%Suma POR FILAS
s = sum(b,2);



[m,pos] = min(b,[], 2);

[m,pos] = min(b,[], 1);




b = randi(4,4,4)

%VARIAR TAMAÑO
%Pasar una matriz a una de 8 filas y 2 columnas
d = reshape(b,8,2);
%pasar una matriz a una de 8 columnas
d = reshape(b,[],8);
%CREAR VECTOR COLUMNA APARTIR DE UNA MATRIZ
b(:);
%EQUIVALENTE
d = repmat(a.',1,5);


%BUCLES
for ii = 0:5:30
ii
end

snrs = 0:5:30;
for ii = snrs
ii
end

for ii = 1:length(snrs)
snrs(ii)
end


%Números complejos:
%DEVEMOS NO CAMBIAR LOS VALORES DE I Y J
comp = 4 + 4*j; CUIDADO !!!
%AQUI SE COJE J COMO NUMEROS IMAGINARIOS Y NO EL VALOR QUE HAYAMOS
%ESTABLECIDO
comp = 4 + 1j.*4;
comp = randn(3,3) + 1j.*randn(3,3);
abs(comp)
angle(comp)
real(comp)
imag(comp)
comp.’
comp


