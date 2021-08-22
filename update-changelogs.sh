#/bin/bash/sh
echo to update run ./update-changlelogs.sh >Koltons-Changelog.txt
git log --stat --author "bloomkd46" >>Koltons-Changelog.txt

echo to update run ./update-changlelogs.sh >Williams-Changelog.txt
git log --stat --author "StrongMan19" >>Williams-Changelog.txt

git log git log -S"#Update" >Project-Changelog.txt

git add Koltons-Changelog.txt Williams-Changelog.txt Project-Changelog.txt
git commit -m "Updated Koltons-Changelog.txt, Williams-Changelog.txt, and Project-Changelog.txt" >/dev/null

echo Changelogs Successfully Updated
