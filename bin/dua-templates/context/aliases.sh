#!/usr/bin/env bash

# Custom aliases for container internal shell.
#
# Roberto Masocco <robmasocco@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# April 4, 2023

# Add custom, general-purpose aliases here.
# You can also source other files from sub-units included by this project.

alias ls='ls --color=auto'
alias ll='ls -la'
alias valgrind-check='valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes --verbose --log-file=valgrind.out'
