## 개요
2024 서울대학교 공과대학 <공학지식의 실무응용> 수업의 기업체 실전문제 해결형 프로젝트 수행 내용입니다. <br>
본 프로젝트는 회사와의 계약을 통해 진행되었으므로 프로젝트 내용 수록 시 정보의 보안사항을 준수합니다. <br>
<br>

## 프로젝트 소개: Bin Picking Advancement(BPA)
<img src="https://github.com/user-attachments/assets/def23c7d-29eb-4442-893e-70bf7f8344d3" width="400" height="350"/> <br>
생산 라인에서 사용되는 빈 피킹(Bin Picking) 로봇은 3D 카메라로 그 위치를 측정하고 조절합니다. <br>
이를 위해서는 카메라 좌표계와 로봇 팔의 좌표계가 동일해야 하며, 두 좌표계를 통일하기 위해 변환행렬(H)이 사용됩니다. <br>
이 좌표계 통일 방식을 개선하여 **작업자 숙련도가 없어도 캘리브레이션이 원활하게 이루어지도록 하는 것**이 프로젝트의 목표입니다. <br>
<br>

## 1차 목표: 임의의 점에서의 테스트 시행을 통한 변환행렬 계산
$A$: **로봇 팔 좌표계 $\to$ 카메라 좌표계**로 변환하는 미지의 행렬 (즉, $A^{-1}$는 **카메라 좌표계 $\to$ 로봇 팔 좌표계**로 변환하는 행렬) <br>
이때, 변환행렬 $A$의 열벡터를 각각 $A_1$, $A_2$, $A_3$, $A_4$라고 하면 $A=(A_1\ A_2\ A_3\ A_4)$가 성립합니다. <br>
<br>
로봇 팔 좌표계 상의 임의의 좌표를 $(a, b, c)$라고 하고, 로봇 팔이 $x$축, $y$축, $z$축 방향으로 $k$만큼 평행이동한 상황을 생각하면 <br>

   1) 로봇 팔이 초기 위치에 있을 때: $(A_1\ A_2\ A_3\ A_4) · (a, b, c, 1) = a · A_1 + b · A_2 + c · A_3 + A_4 \cdots O$
   2) 로봇 팔이 $x$축 방향으로 $k$만큼 이동: $(A_1\ A_2\ A_3\ A_4) · (a+k, b, c, 1) = (a+k) · A_1 + b · A_2 + c · A_3 + A_4 \cdots X$
   3) 로봇 팔이 $y$축 방향으로 $k$만큼 이동: $(A_1\ A_2\ A_3\ A_4) · (a, b+k, c, 1) = a · A_1 + (b+k) · A_2 + c · A_3 + A_4 \cdots Y$
   4) 로봇 팔이 $z$축 방향으로 $k$만큼 이동: $(A_1\ A_2\ A_3\ A_4) · (a, b, c+k, 1) = a · A_1 + b · A_2 + (c+k) · A_3 + A_4 \cdots Z$
   5) 카메라 좌표계를 통해 측정한 $X$, $Y$, $Z$로부터 $O$을 빼면 각각 $k · A_1$, $k · A_2$, $k · A_3$를 얻습니다.
   6) 5.에서 구한 $A_1$, $A_2$, $A_3$를 1)에 대입하여 $A_4 = O - a · A_1 - b · A_2 - c · A_3$을 구합니다. <br>

위 과정을 통해 로봇 팔 좌표계에서 카메라 좌표계로의 변환행렬을 산출할 수 있으며, 그 역행렬인 $A^{-1}$을 구하면 **카메라 좌표계에서의 좌표만으로도 로봇 팔 좌표계에서의 좌표를 역산할 수 있게** 됩니다. <br>

- [성능 평가](https://github.com/ben020410/2024W-ETS-BPA/issues/1)
- 기존 방식은 캘리브레이션 수행 시 수동 teaching 과정이 필요했지만, 개선된 방법을 사용하면 작업 숙련도가 없어도 원활한 캘리브레이션 수행이 가능합니다.
<br>

## 2차 목표: 3D 카메라 데이터를 이용한 제품 곡률 매칭
최초에 제시되었던 picking 작업의 경우에는 위치 정렬(카메라 좌표계와 로봇 팔 좌표계 상의 위치 일치)만 수행하면 충분했으나, <br>
부품 곡면의 sanding과 같은 작업을 원활히 수행하기 위해서는 좌표계의 회전 방향 또한 일치시켜야 합니다(즉, sanding 기계가 가리키는 방향과 곡면의 법선벡터를 일치). <br>
따라서 input으로 tool이 가리키는 방향과 곡면의 법선벡터가 주어질 때, **로봇 팔이 수행해야 하는 회전에 해당하는 행렬**을 계산하고자 하였습니다. <br>
Rodrigues' Rotation Formula에 의하면, 주어진 벡터 $v$를 회전축 $n$과 회전각 $\theta$에 대해 회전시킨 벡터 $v'$은 다음과 같이 계산됩니다. <br>
```math
v' = \left[ I + \sin{\theta} K + (1 - \cos{\theta}) K^2 \right] v, \quad K = \begin{bmatrix} 0 & -n_z & n_y \\ n_z & 0 & -n_x \\ -n_y & n_x & 0 \end{bmatrix}
```
<br>

이 공식을 본 목표에 적용시키면, $v$와 $v'$의 값을 알 때 두 벡터의 변환 과정에 포함되는 행렬 $K$를 계산하는 것입니다. <br>
그러나 변환 과정에서 발생하는 회전각 $\theta$에는 큰 제약이 없다고 판단하여, $\theta=\pi$라고 설정해 계산을 간단히 하였습니다. <br>
<br>
위 과정을 통해 계산한 회전행렬은 다음과 같은 공식을 사용하여 로봇 팔이 수행해야 하는 회전 동작으로 재차 변환하였습니다. <br>
```math
R = \begin{bmatrix} r_{11} & r_{12} & r_{13} \\ r_{21} & r_{22} & r_{23} \\ r_{31} & r_{32} & r_{33} \end{bmatrix} \quad \leftrightarrow \quad \begin{align} \beta=-\arcsin(r_{31}) \\ \alpha=\arccos(r_{11}/\cos\beta) \\ \gamma=\arccos(r_{33}/\cos\beta) \end{align}
```

- [1차 성능 평가](link)
- [2차 성능 평가](link)

<br>

## 3차 목표:
<br>

## Future Improvements



<br>

## References
F. C. Park and K. M. Lynch. (2016). *Introduction to Robotics: Mechanics, Planning, and Control*. Northwestern University.
<br>
