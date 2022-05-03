#!/bin/bash

#Colours
greenColour="\e[0;32m\033[1m"
endColour="\033[0m\e[0m"
redColour="\e[0;31m\033[1m"
blueColour="\e[0;34m\033[1m"
yellowColour="\e[0;33m\033[1m"
purpleColour="\e[0;35m\033[1m"
turquoiseColour="\e[0;36m\033[1m"
grayColour="\e[0;37m\033[1m"

# Defaults
python_version=3.8
branch=dev


SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
CONDA_ENV_PATH="$(dirname "$(dirname "$SCRIPT_DIR")")"
CONDA_ENV_FILE="$CONDA_ENV_PATH/conda-environment.yml"
ln -f $CONDA_ENV_FILE . # Hard link to conda environment file

function helpPanel(){
    echo -e "${redColour}[!] Usage: This script must be executed with the PyHelios dockerfile present in the same directory. ${endColour}\n"
    echo -e "\t${grayColour}[-v]${endColour}${yellowColour} Python version to be used in the installation (e.g. 3.9)${endColour} ${blueColour} Default: $python_version ${endColour}\n"
    echo -e "\t${grayColour}[-b]${endColour}${yellowColour} Repository branch or tag to be used (main, dev, v1.1.0)${endColour} ${blueColour} Default: $branch ${endColour}\n"
    echo -e "\t${grayColour}[-h]${endColour}${yellowColour} Show this panel.${endColour}\n"
    exit 0
}

function buildImageName(){
    python_version_nodot="${python_version//./}"
    if [[ $branch =~ v[0-9]\.[0-9]\.[0-9] ]]
    then branch="${branch//v/}"
    fi # Check if a tag is being used and remove v
    image_name="helios-py${python_version_nodot}:${branch}"
}

function buildImage(){
    docker build -t $image_name --build-arg pyv=$python_version --build-arg branch=${branch} .
}

function checkBuild(){
echo -e "${grayColour}Helios++ Image Builder. Run with [-h] to see configuration options.${endColour}"
echo -e "${grayColour}Helios++ image will be built with the following parameters:${endColour}"
echo -e "\t${yellowColour}Python version: ${endColour}${blueColour}$python_version ${endColour}"
echo -e "\t${yellowColour}Branch: ${endColour}${blueColour}$branch${endColour}"
echo -e "\t${yellowColour}Image name: ${endColour}${blueColour}$image_name${endColour}"
echo -e "${greenColour}Build PyHelios image?${endColour}"
read -p "(y/n)" -n 1 -r
echo

if [[ ! $REPLY =~ ^[Yy]$ ]]
then
    [[ "$0" = "$BASH_SOURCE" ]] && echo "Exiting..." && exit 1 || return 1 # handle exits from shell or function but don't exit interactive shell
fi
}


# MAIN PROGRAM

# Get Opts
while getopts "v:b:h" arg; do
    case "$arg" in
        v) python_version=$OPTARG;;
        b) branch=$OPTARG;;
        h) helpPanel;;
        *) echo -e "${redColour}Invalid flag${endColour}"
    esac
done

buildImageName;
checkBuild;
buildImage;