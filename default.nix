{ pkgs ? import <nixpkgs> { }
, stdenv
, mkRosPackage
, robonomics_comm
}:

mkRosPackage rec {
  name = "${pname}-${version}";
  pname = "streamr_agent";
  version = "0.1.0";

  src = ./.;

  propagatedBuildInputs = [ robonomics_comm ];

  meta = with stdenv.lib; {
    description = "An example of using Robonomics with different painment token";
    homepage = http://github.com/vourhey/streamr_agent;
    license = licenses.bsd3;
    maintainers = with maintainers; [ vourhey ];
  };
}
