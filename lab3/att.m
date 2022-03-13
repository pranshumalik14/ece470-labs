% returns the torque on joints due to the attractive potential
function tau = att(q, qdes, myrobot)

% compute the distance between the current and desired joint positions
oq    = zeros(3, size(q,2));
oqdes = zeros(3, size(q,2));
odiff = zeros(3, size(q,2));

for i = 1:size(q,2)
    oqi        = forward(q(1:i)', myrobot);
    oq(:,i)    = oqi(1:3, 4);
    oqdesi     = forward(qdes(1:i)', myrobot);
    oqdes(:,i) = oqdesi(1:3, 4);
    odiff(:,i) = oq(:,i) - oqdes(:,i);
end

% compute attractive force
d    = 1.0; % attraction saturation distance (m)
Fatt = -odiff;

for i = 1:size(odiff, 2)
     if norm(odiff(i)) >= d
        Fatt(i) = Fatt(i) * d/norm(odiff(i)); % normalize (saturate)
    end
end

% compute torque
tau = zeros(6,1);
Jv  = myrobot.jacob0(q, 'trans'); % velocity Jacobian at current joint config

for i = 1:size(Jv, 2)
    tau = tau + ([Jv(:,1:i) zeros(3, size(Jv, 2)-i)]' * Fatt(:,i));
end

tau = (tau/norm(tau))';

end