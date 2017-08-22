FILE(REMOVE_RECURSE
  "CMakeFiles/test.dir/example/test.cc.o"
  "example/test.pdb"
  "example/test"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/test.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
