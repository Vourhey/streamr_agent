{ rev    ? "2344de470bef43f9d460f98db72e87e5f4b73dab"             # The Git revision of nixpkgs to fetch
, sha256 ? "0cckyla28gbdgfjpvyciz754wycb8agni7az63z7bbw5a2hqwhk3" # The SHA256 of the downloaded data
}:

builtins.fetchTarball {
  url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
  inherit sha256;
}
