# storagy-repo-1

## HOW TO START

### Git 로컬 설정

* git 설정 진행

`git config --global user.name "Your Name"`

`git config --global user.email "your.email@example.com"`

* 설정 확인

`git config --list`


### Git 로그인 설정

* SSH 키 생성

`ssh-keygen -t ed25519 -C "your.email@example.com"`

* SSH 공개키 복사

`cat ~/.ssh/id_ed25519.pub`

* 공개키 복사 및 Git 계정에 추가
    * Cat 명령어로 표시되는 키 복사
    * github.com/settings/keys 로 접근
    * New SSH Key
    * 이름 입력 후 공개키 복사 > Add SSH Key

* 키 테스트

`ssh -T git@github.com`

### Repository 클론 및 리모트 설정

* git Clone

`git clone git@github.com:addinedu-advance-3rd/storagy-repo-1.git`

* 리모트 설정

`git remote set-url origin git@github.com:addinedu-advance-3rd/storagy-repo-1.git`

