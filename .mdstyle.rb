# Refer to https://github.com/markdownlint/markdownlint/blob/main/docs/RULES.md
all

# This allows for a correct visualization on BitBucket
rule 'MD007', indent: 4 # Unordered list indentation

# This allows for a correct numbering on BitBucket
rule 'MD029', style: "ordered" # Ordered list item prefix

# This allows for any line length
exclude_rule 'MD013' # Line length
