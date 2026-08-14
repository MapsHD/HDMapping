# Repository management utilities

- check_clang_format.py - script for CI to check whether clang-format was ran before commiting
- run_clang_format.py - runs clang-format on all project files
- submodules_sync_update.py - performs ```git submodule sync  && git submodule  update --init --recursive``` in case repository was cloned without ```--recursive``` option
- update_submodules.py - updates all submodules to pinned branch and performs code fetch (for submodules without explicit branch pin it will report no pinned branch and do nothing)