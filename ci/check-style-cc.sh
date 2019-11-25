fix_me="" # as parameter
must_fix="" # as parameter
exit_value=0

# https://stackoverflow.com/questions/10804732/what-is-the-difference-between-and-in-regex

# Set constants for maximum line length.
readonly LENGTH=100
readonly TABSIZE=4
readonly TABS="$((LENGTH/TABSIZE))"
readonly CHARS="$((LENGTH-TABS*TABSIZE))"


# Returns all $2-matching files containing lines matching the regex $1.
get_files() {
  local ign="\(\.cc\)"
  #git ls-files | grep -v -i "$ign" | grep "$2" | xargs grep -l -P "$1"
  ls | grep -i "$ign" | grep "$2" | xargs grep -l -P "$1"
}


# Warns of $1 if any $5-matching file contains lines matching the regex $2.
# If -f was set, $3 is executed with each erroneous file as an argument.
# use $4 to print files or lines
style_check() {
  echo "Checking for $1..."
  local files="$(get_files "$2" "${5:-^}")"

  if [ -n "$files" ]; then
    if [ -n "$fix_me" ] && [ -n "$3" ]; then
      for file in $files; do
          echo "  Fixing $file..."
          eval "$3" "$file"
      done
    else
      # Echo lines where errors occured (INcluding comments)
      if [[ "$4" = "0" ]]; then
        echo "The following lines contain $1. Please fix that."
        grep -H -n -P "$2" $files | awk '$0="  "$0'
      # Echo lines where errors occured (EXcluding comments)
      elif [[ "$4" = "1" ]]; then
        px="^([^:]*:[^:]*:\s*\w)"
        # pz="[^"$2"].*:"$2
        # grep -H -n -P "$2" $files | grep -P "$px" | grep --color="always" -P "$pz"|awk '$0=" "$0'
        grep -H -n -P "$2" $files | grep  -P "$px" | awk '$0="  "$0'
      # Echo all files containing errors
      else
        echo "The following files contain $1. Please fix that."
        echo "$files"
      fi
      exit_value=1
    fi
  fi
}

###################################################################################################
## Semicola
###################################################################################################

# double semicola
style_check "lines with too much semicola" \
  '^(?!.*for\s*\(.*)(?=.*;.*;.*)' "0" "0"
echo

# atleast one semicolon
#style_check "lines which do not end with atlest one semicolon" \
#  '^(?!.*;\s*)(?![\#\{\}]|namespace|s*)' "0" "0"
#echo

###################################################################################################
## Whitespace
###################################################################################################

# 1. never more then one whitespace
style_check "lines with more then one whitespace" \
  '[\S]\s\s' "0" "0"
echo

################
## Roundbrace ##
################

# 2.1 before ( and after ) should be a whitespace if it is an if or for
# TODO not jet working for functions and function calls (they are found)
style_check "lines with missing whitespace before ( or after )" \
  '[^\s]\(|\)[^\s;]' "0" "0"
echo

# 2.2 before ( and after ) MUST NOT be a whitespace
style_check "lines with whitespace after ( or before )" \
  '\(\s[^;]|[^;]\s\)' "0" "0"
echo

################
## Curlybrace ##
################

# 3.1 before { and after } should be a whitespace
# TODO
style_check "lines with missing whitespace before { or after }" \
  '\S\{|\}[^\s;]' "0" "0"
echo

# 3.2 before } and after { MUST NOT be a character
style_check "lines with character after { or before }" \
  '\{\s*[^\s\}]|[^\s\{]\s*\}' "0" "0"
echo

#####################
## comma and colon ##
#####################

# 4.1 whitespace after comma or colon
style_check "lines with missing whitespace after colon" \
  '[;][\S]' "0" "0"
echo

# 4.2 whitespace after comma
style_check "lines with missing whitespace after comma" \
  '[,][\S]' "0" "0"
echo

###############
## Operators ##
###############

# 5. whitespace around operators like == = += etc.
#style_check "lines with missing whitespace after //" \
#  '[/][/][^\s/]' "0" "0"
#echo

################
## Commentary ##
################

# 6. whitespace after //
style_check "lines with missing whitespace after //" \
  '[/][/][^\s/]' "0" "0"
echo

# outdated (but works)
#style_check "lines with too much or few whitespace" \
#  '^(?=\s*for\s*\(.*)(?!\s*for \(\S* \S* \S* \S*; \S* \S* \S*; \S*\) \{)' "0" "0"
#echo

###################################################################################################
## Curlybrace
###################################################################################################



###################################################################################################
## Lineendings
###################################################################################################

# Check for common style mistakes.
style_check "lines with whitespace or DOS-style \\r at the end of the line" \
  '[ \t\r]+$' 'sed -i "s/[ \t\r]\+$//"'
echo

###################################################################################################
## Linelength
###################################################################################################

style_check "lines longer than $LENGTH (each tab counts as $TABSIZE chars)" \
  "(\\t|.{$TABSIZE}){$TABS}.{$CHARS}" "0" "0"
echo

echo
if [ "$exit_value" -eq 1 ]; then
  echo "Errors ocurred. Please fix them."
  if [ -n "$fix_me" ]; then
    echo "Some errors could not be fixed. Please fix them manually."
  fi
fi

if [ -n "$must_fix" ]; then
  exit "$exit_value"
fi
