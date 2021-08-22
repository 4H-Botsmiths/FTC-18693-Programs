#/bin/bash/sh
echo to update run ./update-changlelogs.sh >Koltons-Changelog.txt
git log --stat --author "bloomkd46" --grep="Updated Koltons-Changelog.txt, Williams-Changelog.txt, and Project-Changelog.txt" --invert-grep >>Koltons-Changelog.txt

echo to update run ./update-changlelogs.sh >Williams-Changelog.txt
git log --stat --author "StrongMan19" --grep="Updated Koltons-Changelog.txt, Williams-Changelog.txt, and Project-Changelog.txt" --invert-grep >>Williams-Changelog.txt

echo to update run ./update-changlelogs.sh, to make commits apear add Update: to the start of them >Project-Changelog.txt
git log --grep="Update:" >>Project-Changelog.txt

git add Koltons-Changelog.txt Williams-Changelog.txt Project-Changelog.txt
git commit -m "Updated Koltons-Changelog.txt, Williams-Changelog.txt, and Project-Changelog.txt" >/dev/null
git push

echo Changelogs Successfully Updated
