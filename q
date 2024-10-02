[33mcommit 050e4af7d52b642ec6e44a65f736c33276863700[m[33m ([m[1;36mHEAD -> [m[1;32mmaster[m[33m, [m[1;31mbuf/master[m[33m)[m
Author: ryung_robot <ryung9514@naver.com>
Date:   Wed Oct 2 23:26:11 2024 +1100

    이제 ㄹㅇ 작동완료됨!!
    나중에 코드 정리만 하면 끝

[33mcommit 24c989fa017d953c14a5bdbfe9bb638b6b494b92[m
Author: ryung_robot <ryung9514@naver.com>
Date:   Tue Oct 1 20:00:15 2024 +1100

    mid commit

[33mcommit 8db428f38ff7df6c26dedaede8147ed225e16823[m
Author: ryung_robot <ryung9514@naver.com>
Date:   Tue Oct 1 17:51:36 2024 +1100

    odrive 노드를 하나로 합쳐서 충돌 문제 해결 포트고정화 완료
    게인튜닝만 하면 최소요건 만족
    ㅂ

[33mcommit fe9b07a0b0c8f3657c0f98fe45be537646efdbbd[m
Author: ryung_robot <ryung9514@naver.com>
Date:   Tue Oct 1 03:16:26 2024 +1100

    오드라이브 각각 제어까지는완료, 그러나 오드라이브가 충돌되어서 한번에 안돌아감
    무게 중심이 높아서 p를 좀 더 낮춰야할 것으로 보임!

[33mcommit 46792d3e5efbb3354d381f94bd165bc4c2bdd7ba[m
Author: ryung_robot <ryung9514@naver.com>
Date:   Sun Sep 29 02:56:40 2024 +1100

    real end

[33mcommit a7ae8620ab4aa718ef18ebf7220ffd30060cb51d[m
Merge: 3b141ea 80b10bf
Author: ryung_robot <ryung9514@naver.com>
Date:   Sat Sep 28 16:26:40 2024 +1100

    2-wheel.ver
    torque control
    
    Merge branch 'master' of https://github.com/Ryung-coding/ICT2024_surveillance_balancing_robot

[33mcommit 3b141eaecf37699bd25b005265fc3a29b1072e77[m
Author: ryung_robot <ryung9514@naver.com>
Date:   Fri Aug 30 01:37:47 2024 +0900

    pull용 로그업데이트

[33mcommit 80b10bfcef10716baf76695cdf18725bc12b6222[m[33m ([m[1;31mmain/master[m[33m)[m
Author: Ryung <99808176+Ryung-coding@users.noreply.github.com>
Date:   Fri Aug 30 01:34:38 2024 +0900

    Update README.md

[33mcommit 59457b695d17191cac1f321d0cc2864aeb6ac584[m
Author: ryung <tlsgksfbd@seoultech.ac.kr>
Date:   Fri Aug 30 01:33:15 2024 +0900

    웹소켓 및 로컬파일 저장까지 최종 ㄹㅇ 이게 끝나네 ㄹㅇ ㅋㅋㅋㅋ

[33mcommit b1c0f7c4ab44eafa1d6a70b8f77eab74a235a0d0[m
Author: ryung <tlsgksfbd@seoultech.ac.kr>
Date:   Fri Aug 30 00:41:16 2024 +0900

    영상처리 및 웹소켓까지 완료

[33mcommit d70d6ab9c110b4c9f7dddcf7b61b4ced257472f6[m
Author: ryung_robot <ryung9514@naver.com>
Date:   Thu Aug 29 23:00:54 2024 +0900

    영상처리/web 제외 최종 게인튜닝 및 제어구조 완료 Note*시리얼 포트가 바뀌는 이슈 있으니 확인하고 해야함

[33mcommit 3f758542eb85e1abbbc1f911a5ce31c11bae1486[m
Author: ryung <tlsgksfbd@seoultech.ac.kr>
Date:   Sat Aug 24 16:33:30 2024 +0900

    kill 및 web update 추가

[33mcommit 63bc3192c937a0998c621f7bc7900aa112e58cbd[m
Author: ryung_robot <ryung9514@naver.com>
Date:   Sat Aug 24 15:41:56 2024 +0900

    게인튜닝1차최종

[33mcommit 01f5961c3ee9092679fb014b7d1c784a0398c2c6[m[33m ([m[1;31morigin/master[m[33m, [m[1;31morigin/HEAD[m[33m)[m
Author: ryung <tlsgksfbd@seoultech.ac.kr>
Date:   Sat Aug 24 01:04:44 2024 +0900

    자세제어까지끝

[33mcommit 61a80183d4409586b9f592035d70a147daa89231[m
Author: ryung <tlsgksfbd@seoultech.ac.kr>
Date:   Fri Aug 23 21:57:28 2024 +0900

    각속도부분업데이트 전PID부분

[33mcommit 958acdbc8d8bf1140d5752c7600289a5da77dab9[m
Merge: ef11ca7 7c744c2
Author: ryung <tlsgksfbd@seoultech.ac.kr>
Date:   Thu Aug 22 23:52:33 2024 +0900

    충돌로 인하여 pull
    Merge branch 'master' of https://github.com/Ryung-coding/buf

[33mcommit ef11ca71ae129cf06ed3a52f0b9c46e80eab0fa1[m
Author: ryung <tlsgksfbd@seoultech.ac.kr>
Date:   Thu Aug 22 23:51:38 2024 +0900

    2024.08.22 준최종게인, 축이 갈려서 D게인을 실제로 잘 반영을 못하고 있었음 PID2의 경우에는 반응성을 올리는 양태, PID1은 자세를 잡는 것이 우세이다

[33mcommit 7c744c27fba42591dfaef18b43e1345d587c4623[m
Author: Ryung <99808176+Ryung-coding@users.noreply.github.com>
Date:   Thu Aug 22 21:49:21 2024 +0900

    Create README.md

[33mcommit bd99a84e6adcfc57db19e81ff70c110839dae439[m
Author: ryung <tlsgksfbd@seoultech.ac.kr>
Date:   Thu Aug 22 21:48:13 2024 +0900

    GPS코드 추가전 최종

[33mcommit 4b6f0a2035810ac6d4e6f84735cf5588e12411e1[m
Author: ryung_robot <ryung9514@naver.com>
Date:   Tue Aug 20 14:06:48 2024 +0900

    워크스페이스 이동
