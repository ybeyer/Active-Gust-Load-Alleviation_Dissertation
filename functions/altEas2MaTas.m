function [Ma,TAS] = altEas2MaTas(alt,EAS)
atm = isAtmosphere(alt);
TAS = EAS*sqrt(1.225/atm.rho);
Ma = TAS/atm.a;
end
