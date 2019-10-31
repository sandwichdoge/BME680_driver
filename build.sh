ROOT=$PWD
SCRIPT_DIR=`dirname "$0"`
SCRIPT_DIR="$ROOT/$SCRIPT_DIR/"

cd ../Work/01.uhes_protocol/linux/script/

. ./show_help.sh
. ./utils.lib
parse_opts "$@"

cd $SCRIPT_DIR

make
    