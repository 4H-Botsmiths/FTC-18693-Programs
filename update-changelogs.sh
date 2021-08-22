#/bin/bash/sh
echo to update run ./update-changlelogs.sh >Koltons-Changelog.txt
git log --author="bloomkd46" --stat >>Koltons-Changelog.txt
#--grep="Updated Koltons-Changelog.txt, Williams-Changelog.txt, and Project-Changelog.txt" --invert-grep

echo to update run ./update-changlelogs.sh >Williams-Changelog.txt
git log --author="StrongMan19" --stat >>Williams-Changelog.txt
#--grep="Updated Koltons-Changelog.txt, Williams-Changelog.txt, and Project-Changelog.txt" --invert-grep

echo to update run ./update-changlelogs.sh, to make commits apear add Update: to the start of them >Project-Changelog.txt
git log --grep="Update:" >>Project-Changelog.txt

git add Koltons-Changelog.txt Williams-Changelog.txt Project-Changelog.txt
git commit -m "Updated Koltons-Changelog.txt, Williams-Changelog.txt, and Project-Changelog.txt" >/dev/null


echo Changelogs Successfully Updated
