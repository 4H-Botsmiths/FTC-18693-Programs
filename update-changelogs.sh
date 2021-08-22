#/bin/bash/sh
echo to update run ./update-changlelogs.sh >Koltons-Changelog.txt
git log --stat --author "bloomkd46" >>Koltons-Changelog.txt

echo to update run ./update-changlelogs.sh >Williams-Changelog.txt
git log --stat --author "StrongMan19" >>Williams-Changelog.txt

git add Koltons-Changelog.txt Williams-Changelog.txt
git commit -m "Updated Koltons-Changelog.txt and Williams-Changelog.txt" >/dev/null

echo Changelogs Successfully Updated
