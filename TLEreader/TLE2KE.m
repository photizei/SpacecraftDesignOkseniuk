%TLE 2 Keplerian Elements converter
% by Kevin Okseniuk for AA236
close all; clear all;
%% Import TLE from text file
filename = 'TLE.txt';
TLE_file = fopen(filename);
TLE = struct(); %make structure

%% Parse TLE
% Name
scname = fgets(TLE_file);
TLE.spacecraft_name = scname(1:(end-2)); %remove carrier
% Line 1
firstline = split(fgets(TLE_file));
TLE.sat_ID = firstline{2} ;
TLE.international_designation = firstline{3};
TLE.epoch_year = firstline{4}(1:2);
TLE.epoch_dayhour = firstline{4}(3:end);
TLE.dMdt = firstline{5};
TLE.dMdt2 = ['.' firstline{6}];
TLE.BSTAR = ['.' firstline{7}];

% Line 2
secondline = split(fgets(TLE_file));
TLE.inclination = secondline{3};
TLE.RAAN = secondline{4};
TLE.eccentricity = ['.' secondline{5}];
TLE.arg_of_perigee = secondline{6};
TLE.mean_anomaly = secondline{7};
TLE.mean_motion = secondline{8};
TLE.revs_at_epoch = secondline{9};

%% Conditioning for input into OE2ECI function
e = str2double(TLE.eccentricity);
i = str2double(TLE.inclination);
Om = str2double(TLE.RAAN);
w = str2double(TLE.arg_of_perigee);
ME = 5.9742E24; % kg
G = 6.67408E-11; % m3 kg-1 s-2
mu = G * ME;
n_km_sec = str2double(TLE.mean_motion) * 2*pi / (60*60*24) ;
a = (mu/(n_km_sec^2))^(1/3);
nu = rad2deg(mean2true(str2double(TLE.mean_anomaly),str2double(TLE.eccentricity), 1e-10));
nu_2 = rad2deg(nuFromM(str2double(TLE.mean_anomaly),str2double(TLE.eccentricity), 1e-10));
nu_3 = eqofcenter(str2double(TLE.mean_anomaly),str2double(TLE.eccentricity));
%% Convert to ECI
[rvec_ECI, vvec_ECI] = oe2eci(a, e, i, Om, w, nu_3);

%% Auxiliary Functions
% Note: all this functions were written by Kevin Okseniuk for AA279A,
% Winter 2020
function [rvec_ECI, vvec_ECI] = oe2eci(a, e, i, Om, w, nu)
    % Function to transform from orbital elements to Earth-Centered Inertial
    % frame
    % Coded for AA279A by Kevin Okseniuk

    %% Inputs
    % a - semimajor axis, m
    % e - eccentricty
    i = deg2rad(i); % i - inclination, input in deg
    Om = deg2rad(Om); % Om - RAAN, input in deg
    w = deg2rad(w); % w - argument of periapsis, input in deg
    nu = deg2rad(nu); % nu - true anomaly, input in  deg
    % [TO BE IMPLEMENTED] UnitsSettings: .distance : 'km' or 'm' or 'DU'
    % .angles : 'rad' or 'deg'

    %% Constants
    AU = 1.49597870700E11; %m
    ME = 5.9742E24; % kg
    RE = 6.3781E6; % meters
    global G; G = 6.67408E-11; % m3 kg-1 s-2
    aE = 1 * AU;
    mu = G * ME;

    % Define general parameters
    rp = a * (1 - e); ra = a * (1 + e);
    E = - mu / (2 * a);
    h = sqrt(((e ^ 2) - 1) * (mu ^ 2) / (2 * E));
    p = h ^ 2 / mu;

    %Define vector in perifocal coordinates, PQR
    rmag = p / (1 + e * cos(nu));
    rvec_pf = rmag * [cos(nu), sin(nu), 0]';
    vvec_pf = (mu / h) * [- sin(nu), (e + cos(nu)), 0]';

    % Compute rotation matrices
    R3w = R3rot(w);
    R1i = R1rot(i);
    R3Om = R3rot(Om);

    %% Rotation cases
    inclined = i > 1.001 * eps;
    circular = e < 1.001 * eps;

    if i < 0 || e < 0
        error('bad inclination and/or eccentricity!')
    end

    switch inclined
        case true
            switch circular
            case true
                R_ECI_2_PF = R1i * R3Om; % Inclined and circular
            case false
                R_ECI_2_PF = R3w * R1i * R3Om; % Inclined but NOT circular
        end
        case false
          switch circular
          case true
              R_ECI_2_PF = eye(3); % Not inclined but circular
          case false
              R_ECI_2_PF = R3w * R3Om; % Not inclined and NOT circular
      end
  end

  % Apply appropriate rotation matrix from ECI to PF
  rvec_ECI = (R_ECI_2_PF)'*rvec_pf;
  vvec_ECI = (R_ECI_2_PF)'*vvec_pf;

  end

  function [R3] = R3rot(om)
      %input in radians
      R3 = [cos(om), sin(om), 0; ...
       - sin(om), cos(om), 0; ...
      0, 0, 1];

      % Vrot = R*V;
  end

  function [R1] = R1rot(om)
      %input in radians
      R1 = [1, 0, 0; ...
      0, cos(om), sin(om); ...
      0, - sin(om), cos(om)];

      % Vrot = R*V;
  end

  function f = mean2true(M, e, tol)
% mean2true solves Kepler's equation for true anomaly
%
%   Note: This function uses a Newton-Raphson method to numerically compute
%   the correct value for f
%
% Inputs:
%     M - mean anomaly [rad]
%     e - eccentricity of orbit
%   tol - tolerance for Newton-Raphson iterator
%
% Outputs:
%     f - true anomaly [rad]

if nargin == 2
    tol = 10e-10;
end

E = mean2ecc(M,e,tol);
f = ecc2true(E,e);

  end

  function E = mean2ecc(M, e, tol)
% mean2ecc solves Kepler's equation for eccentric anomaly
%
%   Note: This function uses a Newton-Raphson method to numerically compute
%   the correct value for E
%
% Inputs:
%     M - mean anomaly [rad]
%     e - eccentricity of orbit
%   tol - tolerance for Newton-Raphson iterator
%
% Outputs:
%     E - eccentric anomaly [rad]

M = mod(M, 2*pi);

if M == 0 | M == pi
    % Return the known solutions (trivial)
    E = M;
else
    % Set up the problem based on an initial guess
    E0 = M;
    d = -(E0 - e*sin(E0) - M)/(1 - e*cos(E0));

    % Loop until the solution converges
    while abs(d) > tol
        E1 = E0 + d;

        d = -(E1 - e*sin(E1) - M)/(1 - e*cos(E1));

        E0 = E1;
    end

    E = E0;
end

  end

  function f = ecc2true(E, e)
% ecc2true Computes true anomaly given eccentric anomaly and eccentricity
%
% Inputs:
%       E - eccentric anomaly [rad]
%       e - eccentricity of orbit
%
% Outputs:
%       f - mean anomaly [rad]

E = mod(E, 2*pi);

f = acos((cos(E) - e)/(1 - e*cos(E)));

% Make sure E sits in the correct semi-plane
if E > pi
    f = 2*pi - f;
end

  end

  % Orbit eccentric anomaly, Kepler's equation keplers equation
% Richard Rieber
% 1/23/2005
% rrieber@gmail.com
% 
% Revision 8/21/07: Fixed typo in line 38 if statement
%                   Added H1 line for lookfor functionality
%
% function E = CalcEA(M,ecc,tol)
% 
% Purpose: Solves for eccentric anomaly, E, from a given mean anomaly, M,
%          and eccentricty, ecc.  Performs a simple Newton-Raphson iteration
%
% Inputs: o M   - Mean anomaly in radians.
%         o ecc - Eccentricity of the orbit.
%         o tol - a tolerance at which to terminate iterations; Default
%                 is 10^-8 radians. [OPTIONAL]
%
% Outputs: o E  - The eccentric anomaly in radians.
%
% E = CalcEA(M,ecc) uses default tolerances
%
% E = CalcEA(M,ecc,tol) will use a user specified tolerance, tol
% 
function E = CalcEA(M,ecc,tol)
%Checking for user inputed tolerance
if nargin == 2
    %using default value
    tol = 10^-8;
elseif nargin > 3
    error('Too many inputs.  See help CalcEA')
elseif nargin < 2
    error('Too few inputs.  See help CalcEA')
end
if (M > -pi && M < 0) || M > pi
    E = M - ecc;
else
    E = M + ecc;
end
Etemp  = E + (M - E + ecc*sin(E))/(1-ecc*cos(E));
Etemp2 = E;
while abs(Etemp - Etemp2) > tol
    Etemp = Etemp2;
    Etemp2 = Etemp + (M - Etemp + ecc*sin(Etemp))/(1-ecc*cos(Etemp));
end
E = Etemp2;

end

% Richard Rieber
% September 27, 2006
% rrieber@gmail.com
% 
% Revision 9/25/07 - Fixed a grusome error in the default tolerance.
%                    Changed from 10^8 radians to 10^-8 radians.  Whoops.
%
% function nu = nuFromM(M,ecc,tol)
% 
% Purpose:  This function calculates the true anomaly (nu) of a position in an orbit given
%           the mean anomaly of the position (M) and the eccentricity (ecc) of the orbit.
%           This uses another function, calcEA.
%           
% Inputs:  M - mean anomaly of position in radians
%          ecc - eccentricity of orbit
%          tol - A tolerance for calculating the eccentric anomaly (see help calcEA.m)
%                Default is 10^-8 radians [OPTIONAL]
% 
% Output:  nu - true anomaly of position in radians

function nu = nuFromM(M,ecc,tol)

if nargin < 2 || nargin > 3
    error('Incorrect number of inputs, see help nuFromM.m')
elseif nargin == 2
    tol = 10^-8;
end

E = CalcEA(M,ecc,tol);  %Determining eccentric anomaly from mean anomaly

% Since tan(x) = sin(x)/cos(x), we can use atan2 to ensure that the angle for nu
% is in the correct quadrant since we know both sin(nu) and cos(nu).  [see help atan2]
nu = atan2((sin(E)*(1-ecc^2)^.5),(cos(E)-ecc));

end

function nu = eqofcenter(M, e)

nu = M + (2*e - 0.25*(e^3))*sind(M) + (5/4)*(e^2)*sind(2*M) + (13/12)*(e^3)*sind(3*M);

end